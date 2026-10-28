"""Prepared read-only PC displays. Default is offline validation; --start connects.

No firmware/serial/motor/camera-service writes. No telemetry files are created.
The controller monitor reuses its existing in-memory SSH reader and HTTP server.
Only this invocation's own camera SSH tunnel is cleaned up on exit.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import re
import shlex
import socket
import subprocess
import threading
import webbrowser


def validate(config, *, require_log=True):
    missing = []
    for key in ('pi_host', 'pi_user', 'host_key_alias', 'identity_file', 'known_hosts_file', 'monitor_source'):
        if not isinstance(config.get(key), str) or not config[key].strip():
            missing.append(key)
    if require_log and not config.get('remote_control_log'):
        missing.append('remote_control_log')
    if missing:
        raise ValueError('Missing settings: ' + ', '.join(missing))
    if config['pi_host'].startswith('-') or any(c.isspace() for c in config['pi_host']):
        raise ValueError('Invalid pi_host')
    if not re.fullmatch(r'[a-zA-Z0-9_.-]+', config['pi_user']):
        raise ValueError('Invalid pi_user')
    if any(c.isspace() for c in config['host_key_alias']):
        raise ValueError('Invalid host_key_alias')
    for key in ('camera_remote_port', 'camera_local_port', 'monitor_local_port'):
        if type(config.get(key)) is not int or not 1 <= config[key] <= 65535:
            raise ValueError('Invalid ' + key)
    if config['camera_local_port'] == config['monitor_local_port']:
        raise ValueError('Camera and monitor local ports must differ')
    log = config.get('remote_control_log')
    if log and (not isinstance(log, str) or not log.startswith('/') or '\n' in log or '\x00' in log):
        raise ValueError('remote_control_log must be an absolute Pi path')
    for key in ('identity_file', 'known_hosts_file', 'monitor_source'):
        if not Path(config[key]).is_file():
            raise ValueError('File missing: ' + key)


def ssh_base(config):
    args = ['ssh']
    if config.get('ipv6'):
        args += ['-6']
    return args + ['-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
                   '-o', 'HostKeyAlias=' + config['host_key_alias'],
                   '-o', 'UserKnownHostsFile=' + config['known_hosts_file'],
                   '-i', config['identity_file'], '-o', 'ConnectTimeout=5',
                   '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2']


def monitor_command(config):
    return ssh_base(config) + [config['pi_user'] + '@' + config['pi_host'],
        'tail -n 0 -F -- ' + shlex.quote(config['remote_control_log'])]


def camera_command(config):
    forward = f"127.0.0.1:{config['camera_local_port']}:127.0.0.1:{config['camera_remote_port']}"
    # -6 also constrains the local listener and breaks an IPv4 127.0.0.1 -L
    # binding on Windows OpenSSH. The literal destination still selects IPv6.
    base = ssh_base(dict(config, ipv6=False))
    return base + ['-N', '-o', 'ExitOnForwardFailure=yes', '-L', forward,
                               config['pi_user'] + '@' + config['pi_host']]


def camera_status(path, state, config, **details):
    data = dict(state=state, local_port=config['camera_local_port'],
        remote_port=config['camera_remote_port'],
        meaning='SSH forwarding status only; camera HTTP/pose availability is separate', **details)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
    temporary.replace(path)


def run_camera_tunnel(config, stop):
    folder = Path(__file__).parent
    status = folder/'camera_tunnel_status.json'
    error_log = folder/'camera_tunnel.stderr.log'
    # Never replace/kill a tunnel or app that owns this port already.
    try:
        with socket.socket() as probe:
            probe.bind(('127.0.0.1',config['camera_local_port']))
    except OSError as exc:
        camera_status(status,'port_already_in_use',config,error=str(exc),owned=False)
        print('Camera forwarding not started: local port already in use.',flush=True)
        return
    attempt = 0
    while not stop.is_set():
        process = None
        attempt += 1
        try:
            with error_log.open('wb') as errors:
                process = subprocess.Popen(camera_command(config),stdout=subprocess.DEVNULL,stderr=errors,
                    creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
                camera_status(status,'connecting',config,pid=process.pid,attempt=attempt,owned=True)
                listening = False
                while process.poll() is None and not stop.is_set():
                    if not listening:
                        try:
                            with socket.create_connection(('127.0.0.1',config['camera_local_port']),timeout=.2):
                                listening = True
                            camera_status(status,'forward_listening',config,pid=process.pid,attempt=attempt,owned=True)
                            print('Camera SSH forward listening on 127.0.0.1:'+str(config['camera_local_port']),flush=True)
                        except OSError:
                            pass
                    stop.wait(.5)
                if stop.is_set():
                    break
                code = process.returncode
            error = error_log.read_bytes()[-2048:].decode('utf-8',errors='replace')
            camera_status(status,'retrying',config,exit_code=code,error=error,attempt=attempt,owned=True)
            print(f'Camera SSH exited ({code}); retrying in 2 s: {error.strip()}',flush=True)
        except OSError as exc:
            camera_status(status,'retrying',config,error=str(exc),attempt=attempt,owned=True)
            print('Camera SSH failed; retrying: '+str(exc),flush=True)
        finally:
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(2.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(2.)
        stop.wait(2.)
    camera_status(status,'stopped',config,owned=True)


def load_monitor(path):
    spec = importlib.util.spec_from_file_location('oasis_pc_readonly_monitor', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run(config, *, open_browser=False):
    validate(config)
    module = load_monitor(config['monitor_source'])
    monitor, stop = module.Monitor('live'), threading.Event()
    # Bind first: an existing waiting/live monitor is never stopped or replaced.
    server = module.Server(config['monitor_local_port'], monitor)
    tunnel = None
    try:
        tunnel = threading.Thread(target=run_camera_tunnel,args=(config,stop),name='camera-forward',daemon=True)
        tunnel.start()
        reader = threading.Thread(target=module.tail_ssh,
            args=(monitor_command(config), monitor, stop), name='pi-read-only-tail', daemon=True)
        reader.start()
        threading.Thread(target=module.tail_session_ssh,
            args=(module.session_ssh_command(ssh_base(config)+[config['pi_user']+'@'+config['pi_host']],
                  config['remote_control_log']),monitor,stop),
            name='read-only-session-status',daemon=True).start()
        threading.Thread(target=module.camera_status_loop,
            args=(f"http://127.0.0.1:{config['camera_local_port']}/status.json",monitor,stop),
            name='read-only-camera-status',daemon=True).start()
        camera_url = f"http://127.0.0.1:{config['camera_local_port']}/"
        monitor_url = f"http://127.0.0.1:{config['monitor_local_port']}/"
        print('Camera preview: ' + camera_url, flush=True)
        print('Controller monitor: ' + monitor_url, flush=True)
        print('Read-only. Ctrl+C stops only these display connections.', flush=True)
        if open_browser:
            webbrowser.open(camera_url)
            webbrowser.open(monitor_url)
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        if tunnel is not None:
            tunnel.join(3.)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('pc_display_config.json'))
    parser.add_argument('--start', action='store_true', help='Connect only after host/log/ports are verified')
    parser.add_argument('--open-browser', action='store_true', help='Open normal local browser tabs after starting')
    parser.add_argument('--camera-only', action='store_true', help='Supervise camera forwarding without replacing an existing monitor')
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text(encoding='utf-8'))
    validate(config, require_log=args.start and not args.camera_only)
    if not args.start:
        print(json.dumps(dict(mode='offline_preflight', network_started=False,
            remote_control_log_set=bool(config.get('remote_control_log')),
            monitor_url=f"http://127.0.0.1:{config['monitor_local_port']}/",
            camera_url=f"http://127.0.0.1:{config['camera_local_port']}/",
            verified_host_key_alias=config['host_key_alias'],
            note='Port availability and Pi services are not checked in offline mode.'), indent=2))
        return 0
    if args.camera_only:
        stop = threading.Event()
        worker = threading.Thread(target=run_camera_tunnel,args=(config,stop),daemon=True)
        worker.start()
        try:
            while worker.is_alive():
                worker.join(.5)
        except KeyboardInterrupt:
            stop.set()
            worker.join(3.)
    else:
        run(config, open_browser=args.open_browser)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
