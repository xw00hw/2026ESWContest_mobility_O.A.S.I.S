"""Prepare a source/ELF viewer and no-reset attach configuration; no devices.

Run before opening the dedicated workspace. The project deliberately has no
IDE build triggers: use scripts/stm32.ps1 -Action Build -FallbackControl.
"""
from pathlib import Path
import json
import xml.etree.ElementTree as ET

STM = Path(__file__).resolve().parents[1]
PROJECT = STM/'cubeide'
WORKSPACE = Path.home()/'STM32CubeIDE/workspace_ExperimentA'
NAME = 'ExperimentA_CAD_180'
TEMPLATE = Path.home()/'Documents/GitHub/SMART_SEAT/MOTER/firmware/r8_debug'


def configure_attach_connection(launch, stlink_serial=''):
    """Set typed GDB connection fields; launch.mode/port are semihosting."""
    attributes = {
        'org.eclipse.cdt.debug.gdbjtag.core.portNumber': ('intAttribute', '61234'),
        'org.eclipse.cdt.debug.gdbjtag.core.ipAddress': ('stringAttribute', 'localhost'),
        'com.st.stm32cube.ide.mcu.debug.launch.useRemoteTarget': ('booleanAttribute', 'true'),
        'com.st.stm32cube.ide.mcu.debug.launch.mode': ('intAttribute', '0'),
        'com.st.stm32cube.ide.mcu.debug.launch.port': ('stringAttribute', ''),
        'com.st.stm32cube.ide.mcu.debug.stlink.stlink_check_serial_number':
            ('booleanAttribute', 'true' if stlink_serial else 'false'),
        'com.st.stm32cube.ide.mcu.debug.stlink.stlink_txt_serial_number':
            ('stringAttribute', stlink_serial),
    }
    for item in list(launch):
        if item.get('key') in attributes:
            launch.remove(item)
    for key, (kind, value) in attributes.items():
        ET.SubElement(launch, kind, key=key, value=value)


def main():
    PROJECT.mkdir(exist_ok=True)
    root=ET.parse(TEMPLATE/'.project').getroot()
    root.find('name').text=NAME
    root.find('comment').text='Experiment A prebuilt ELF/source viewer. No automatic build or upload.'
    root.find('buildSpec').clear()
    links=root.find('linkedResources'); links.clear()
    for name,kind,path in [('Core',2,STM/'Core'),('build_fallback',2,STM/'build_fallback'),
                           ('scripts',2,STM/'scripts'),('config',2,STM/'config'),
                           ('EXPERIMENT_A_LIVE_TUNING.md',1,STM/'EXPERIMENT_A_LIVE_TUNING.md')]:
        link=ET.SubElement(links,'link')
        ET.SubElement(link,'name').text=name
        ET.SubElement(link,'type').text=str(kind)
        ET.SubElement(link,'location').text=path.as_posix()
    ET.indent(root)
    ET.ElementTree(root).write(PROJECT/'.project',encoding='utf-8',xml_declaration=True)
    cfg=ET.fromstring((TEMPLATE/'.cproject').read_text(encoding='utf-8').replace('R8_Bolster_Debug',NAME))
    for c in cfg.iter('configuration'):
        c.set('description','Prebuilt CMake ELF; build explicitly with stm32.ps1')
        c.set('cleanCommand','')
    for b in cfg.iter('builder'):
        b.set('enableAutoBuild','false'); b.set('enableIncrementalBuild','false'); b.set('enableCleanBuild','false')
    ET.indent(cfg)
    ET.ElementTree(cfg).write(PROJECT/'.cproject',encoding='utf-8',xml_declaration=True)
    launch=ET.parse(TEMPLATE/'R8_Bolster_Debug Debug.launch').getroot()
    elf=(STM/'build_fallback/fallback_control_h723.elf').as_posix()
    for item in list(launch):
        key=item.get('key','')
        if key.endswith('stlink_txt_serial_number') or key.endswith('stopAt') or key.endswith('PROJECT_BUILD_CONFIG_ID_ATTR'):
            launch.remove(item); continue
        if key.endswith('reset_strategy'): item.set('value','no_reset')
        elif key.endswith('watchdog_config'): item.set('value','none')
        elif key in ('org.eclipse.cdt.debug.gdbjtag.core.doReset','org.eclipse.cdt.debug.gdbjtag.core.doHalt',
                      'org.eclipse.cdt.debug.gdbjtag.core.loadImage','org.eclipse.cdt.debug.gdbjtag.core.setBreakpoint',
                      'org.eclipse.cdt.debug.gdbjtag.core.setResume') or key.endswith('stlink_check_serial_number'):
            item.set('value','false')
        elif key.endswith('loadList'):
            item.set('value',json.dumps({'fItems':[{'fIsFromMainTab':True,'fPath':elf,'fProjectName':NAME,
                'fBuildConfigId':'','fUseActiveBuildConfig':False,'fPerformBuild':False,'fDownload':False,
                'fLoadSymbols':True,'fSymbolAddress':'','fDownloadOffsetAddress':'','fStartDownloadAddress':''}]}))
        elif key=='org.eclipse.cdt.launch.PROJECT_ATTR': item.set('value',NAME)
        elif key in ('org.eclipse.cdt.launch.PROGRAM_NAME','org.eclipse.cdt.debug.gdbjtag.core.imageFileName',
                     'org.eclipse.cdt.debug.gdbjtag.core.symbolsFileName'): item.set('value',elf)
        elif key=='org.eclipse.debug.core.MAPPED_RESOURCE_PATHS':item[0].set('value','/'+NAME)
    configure_attach_connection(launch)
    ET.SubElement(launch,'intAttribute',key='org.eclipse.cdt.launch.BUILD_BEFORE_LAUNCH_ATTR',value='0')
    ET.indent(launch)
    ET.ElementTree(launch).write(PROJECT/'Experiment A - attach after verified upload.launch',encoding='utf-8',xml_declaration=True)
    expressions=[f'g_experiment_a_tune.values.{field}' for field in ('yaw_max_mdeg','pitch_max_mdeg',
        'yaw_rate_mdeg_s','pitch_rate_mdeg_s','full_scale_ay_mmps2','pitch_scale_permille')]
    expressions+=['g_experiment_a_tune.request_seq','g_fallback_debug.tuning_revision',
        'g_fallback_debug.tuning_result_code','g_fallback_debug.armed','g_fallback_debug.commanded_pose_mdeg',
        'g_fallback_debug.target_pose_mdeg','g_fallback_debug.commanded_pulse_us','g_fallback_debug.fault']
    (PROJECT/'Live_Expressions.txt').write_text('\n'.join(expressions)+'\n',encoding='ascii')
    state=WORKSPACE/'.metadata/.plugins/com.st.stm32cube.ide.mcu.livewatch'
    state.mkdir(parents=True,exist_ok=True)
    destination=state/'saved_expr.dat'
    if not destination.exists():destination.write_text('\n'.join(expressions)+'\n',encoding='ascii')
    print(json.dumps({'project':str(PROJECT),'workspace':str(WORKSPACE),'expressions':len(expressions),
        'devices_opened':False,'launch_executed':False},indent=2))


if __name__=='__main__':main()
