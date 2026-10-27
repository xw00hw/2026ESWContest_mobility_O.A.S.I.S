"""Preview wire metadata tests with synthetic bytes and loopback HTTP only.

No camera, model, GUI, UART or physical device is opened. Capture timestamps
belong to the encoded image, not the latest queued frame or HTTP receipt time.
"""
import http.client
import json
from pathlib import Path
import sys
import threading
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from oasis_rt.pose_camera import Frame
from oasis_rt.pose_preview import LatestPreview,PreviewServer
from test_pose_preview import Clock,wait_for


class TestPreviewCaptureMetadata(unittest.TestCase):
    def setUp(self):
        self.clock=Clock()

    def latest(self,encoder=lambda job:job.image):
        latest=LatestPreview(encoder=encoder,clock_ns=self.clock)
        self.addCleanup(latest.close)
        return latest

    def frame(self,image=b'image',age_ns=10_000_000):
        return Frame(image,self.clock.ns-age_ns,self.clock.ns,self.clock.ns)

    def test_capture_timestamp_never_refreshed_by_poll_gap_or_expiry(self):
        latest=self.latest()
        self.assertIsNone(latest.snapshot()[2]['preview']['frame_capture_ns'])
        frame=self.frame();latest.submit(frame,None,{'reason':'neutral_required'})
        wait_for(lambda:latest.snapshot()[1]==b'image')
        sequence,jpeg,status=latest.snapshot()
        self.assertEqual(sequence,1)
        self.assertEqual(status['preview']['frame_capture_ns'],frame.capture_ns)
        self.assertTrue(status['preview']['image_fresh'])
        self.clock.ns+=40_000_000
        latest.unavailable({'reason':'no_new_frame'})
        self.assertEqual(latest.snapshot()[2]['preview']['frame_capture_ns'],frame.capture_ns)
        self.clock.ns=frame.capture_ns+300_000_000
        _,jpeg,status=latest.snapshot()
        self.assertIsNone(jpeg)
        self.assertFalse(status['preview']['image_fresh'])
        self.assertEqual(status['preview']['frame_capture_ns'],frame.capture_ns)
        self.assertEqual(status['preview']['frame_age_ms'],300.)
        latest.unavailable({'reason':'camera_missing'})
        self.assertIsNone(latest.snapshot()[2]['preview']['frame_capture_ns'])

    def test_pending_frame_cannot_replace_published_jpeg_timestamp(self):
        entered,release=threading.Event(),threading.Event()
        def encode(job):
            if job.image==b'pending':
                entered.set();release.wait(1.)
            return job.image
        latest=self.latest(encode);self.addCleanup(release.set)
        first=self.frame(b'published')
        latest.submit(first,None,{'reason':'neutral_required'})
        wait_for(lambda:latest.snapshot()[1]==b'published')
        self.clock.ns+=20_000_000
        second=self.frame(b'pending')
        latest.submit(second,None,{'reason':'pose_valid'})
        self.assertTrue(entered.wait(1.))
        sequence,jpeg,status=latest.snapshot()
        self.assertEqual((sequence,jpeg),(1,b'published'))
        self.assertEqual(status['preview']['frame_capture_ns'],first.capture_ns)
        release.set();wait_for(lambda:latest.snapshot()[1]==b'pending')
        sequence,jpeg,status=latest.snapshot()
        self.assertEqual(sequence,2)
        self.assertEqual(status['preview']['frame_capture_ns'],second.capture_ns)


class TestPreviewMultipartHeaders(unittest.TestCase):
    def setUp(self):
        self.clock=Clock()
        self.server=PreviewServer(0,clock_ns=self.clock,encoder=lambda job:job.image,blank=b'placeholder')
        self.addCleanup(self.server.close)

    def connect(self,path):
        connection=http.client.HTTPConnection('127.0.0.1',self.server.port,timeout=1.)
        self.addCleanup(connection.close)
        connection.request('GET',path)
        response=connection.getresponse()
        self.assertEqual(response.status,200)
        return response

    def part(self,response):
        self.assertEqual(response.readline(128),b'--oasisframe\r\n')
        headers={}
        for _ in range(12):
            line=response.readline(512)
            if line==b'\r\n':break
            self.assertTrue(line.endswith(b'\r\n'))
            key,value=line.decode('ascii').strip().split(':',1)
            self.assertNotIn(key.lower(),headers)
            headers[key.lower()]=value.strip()
        else:self.fail('Multipart headers exceeded bounded test budget')
        length=int(headers['content-length'])
        self.assertLessEqual(length,128)
        payload=response.read(length)
        self.assertEqual(response.read(2),b'\r\n')
        self.assertEqual(headers['content-type'],'image/jpeg')
        return headers,payload

    def assert_metadata(self,headers,fresh,capture_ns,sequence):
        self.assertEqual(headers['x-frame-fresh'],str(int(fresh)))
        self.assertEqual(headers['x-capture-ns'],str(capture_ns))
        self.assertEqual(headers['x-frame-sequence'],str(sequence))

    def test_initial_placeholder_and_status_have_no_capture_timestamp(self):
        status=json.loads(self.connect('/status.json').read())['preview']
        self.assertFalse(status['image_fresh'])
        self.assertIsNone(status['frame_capture_ns'])
        response=self.connect('/stream.mjpg')
        headers,payload=self.part(response)
        self.assertEqual(payload,b'placeholder')
        self.assert_metadata(headers,False,0,0)

    def test_fresh_stale_recovered_and_disconnected_parts_are_distinguishable(self):
        first=Frame(b'first-jpeg',self.clock.ns-10_000_000,0,0)
        self.server.submit(first,None,{'reason':'neutral_required'})
        wait_for(lambda:self.server.latest.snapshot()[1]==b'first-jpeg')
        response=self.connect('/stream.mjpg')
        headers,payload=self.part(response)
        self.assertEqual(payload,b'first-jpeg')
        self.assert_metadata(headers,True,first.capture_ns,1)
        # Expiration emits a placeholder without inventing a new frame capture.
        self.clock.ns=first.capture_ns+300_000_000
        headers,payload=self.part(response)
        self.assertEqual(payload,b'placeholder')
        self.assert_metadata(headers,False,0,1)
        status=json.loads(self.connect('/status.json').read())['preview']
        self.assertFalse(status['image_fresh'])
        self.assertEqual(status['frame_capture_ns'],first.capture_ns)
        second=Frame(b'second-jpeg',self.clock.ns-1_000_000,0,0)
        self.server.submit(second,None,{'reason':'pose_valid'})
        wait_for(lambda:self.server.latest.snapshot()[1]==b'second-jpeg')
        headers,payload=self.part(response)
        self.assertEqual(payload,b'second-jpeg')
        self.assert_metadata(headers,True,second.capture_ns,2)
        self.server.unavailable({'reason':'camera_read_timeout'})
        headers,payload=self.part(response)
        self.assertEqual(payload,b'placeholder')
        self.assert_metadata(headers,False,0,3)


if __name__=='__main__':unittest.main()
