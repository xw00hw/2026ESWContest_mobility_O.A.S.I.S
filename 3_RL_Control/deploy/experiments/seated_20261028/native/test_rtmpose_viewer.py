import io
import threading
import unittest

from rtmpose_viewer import Latest, MAX_JPEG, read_frame, frames_loop, status_loop


def part(fresh=1, capture=9_900_000_000, sequence=4, jpeg=b'\xff\xd8test\xff\xd9', extra=b''):
    return (b'--oasisframe\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+
            b'\r\nX-Frame-Fresh: '+str(fresh).encode()+b'\r\nX-Capture-Ns: '+str(capture).encode()+
            b'\r\nX-Frame-Sequence: '+str(sequence).encode()+b'\r\n'+extra+b'\r\n'+jpeg+b'\r\n')


class OneIteration:
    stopped=False
    def is_set(self):return self.stopped
    def wait(self,_seconds):self.stopped=True


class TestNativeViewer(unittest.TestCase):
    def test_parser_shared_annotated_jpeg_and_placeholder_identity(self):
        stream=io.BytesIO(part()+part(fresh=0,capture=0,sequence=5))
        fresh,capture,sequence,jpeg=read_frame(stream)
        self.assertTrue(fresh)
        self.assertEqual(capture,9_900_000_000)
        self.assertEqual(sequence,4)
        self.assertEqual(jpeg,b'\xff\xd8test\xff\xd9')
        self.assertEqual(read_frame(stream)[:3],(False,0,5))

    def test_parser_rejects_missing_oversized_duplicate_or_truncated_records(self):
        invalid=[part().replace(b'X-Frame-Fresh: 1\r\n',b''),
                 part(extra=b'Content-Length: 8\r\n'),part()[:-5],
                 part().replace(b'Content-Length: 8',f'Content-Length: {MAX_JPEG+1}'.encode()),
                 part(fresh=2),part(capture=-1),part(jpeg=b'not a jpeg')]
        for data in invalid:
            with self.subTest(data=data[:70]),self.assertRaises((ValueError,KeyError,EOFError)):
                read_frame(io.BytesIO(data))

    def test_latest_only_no_queue_and_stale_ttl(self):
        clock=[10.]
        latest=Latest(lambda:clock[0])
        for i in range(1000):latest.accept(True,9_900_000_000,i,str(i).encode())
        frame,fresh,age,reason,status=latest.snapshot()
        self.assertTrue(fresh)
        self.assertEqual(frame.sequence,999)
        clock[0]=10.65
        self.assertFalse(latest.snapshot()[1])
        self.assertEqual(latest.snapshot()[3],'frame timed out')

    def test_placeholder_and_disconnect_dim_but_retain_last_good(self):
        latest=Latest(lambda:10.)
        latest.accept(True,9_900_000_000,1,b'good')
        latest.accept(False,0,2,b'black-placeholder')
        self.assertEqual(latest.snapshot()[0].jpeg,b'good')
        self.assertFalse(latest.snapshot()[1])
        latest.accept(True,9_950_000_000,3,b'new-good')
        latest.unavailable('disconnected')
        self.assertEqual(latest.snapshot()[0].jpeg,b'new-good')
        self.assertFalse(latest.snapshot()[1])

    def test_future_or_stale_capture_never_replaces_good(self):
        latest=Latest(lambda:10.)
        latest.accept(True,9_900_000_000,1,b'good')
        for capture in (11_000_000_000,1):
            latest.accept(True,capture,2,b'invalid')
            self.assertFalse(latest.snapshot()[1])
            self.assertEqual(latest.snapshot()[0].jpeg,b'good')

    def test_timeout_does_not_exit_gui_state_or_clear_retained_frame(self):
        latest=Latest(lambda:10.)
        latest.accept(True,9_900_000_000,1,b'good')
        calls=[]
        def timeout(url,timeout):
            calls.append((url,timeout))
            raise TimeoutError('mock missing producer')
        frames_loop('http://127.0.0.1:8081',latest,OneIteration(),timeout)
        self.assertFalse(latest.snapshot()[1])
        self.assertEqual(latest.snapshot()[0].jpeg,b'good')
        self.assertEqual(calls[0][1],.5)

    def test_status_optional_expires_and_never_authorizes_image(self):
        clock=[10.]
        latest=Latest(lambda:clock[0])
        latest.set_status({'reason':'pose_valid'})
        self.assertFalse(latest.snapshot()[1])
        clock[0]=11.
        self.assertEqual(latest.snapshot()[4],{})
        def timeout(*_,**__):raise TimeoutError('status missing')
        status_loop('http://127.0.0.1:8081',latest,OneIteration(),timeout)
        self.assertFalse(latest.snapshot()[1])


if __name__=='__main__':unittest.main()
