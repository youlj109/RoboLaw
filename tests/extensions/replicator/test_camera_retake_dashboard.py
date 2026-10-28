"""Camera retake overlay must never alter first-pass scores or serve unbound media."""
from http.server import ThreadingHTTPServer
import threading
import unittest
from unittest.mock import patch
import urllib.error
import urllib.request

from scripts.extensions.replicator import progress_dashboard as dashboard
from tests.extensions.replicator import test_package_reviewable_physical_demos as fixtures


class CameraRetakeDashboardTests(unittest.TestCase):
    def setUp(self):
        self.fixture=fixtures.ReviewableDemoPackagingTests('test_complete_bundle_keeps_worse_retake_and_all_original_bytes')
        self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        f=self.fixture
        for key,value in (('PHYSICAL_SOURCE',f.rel(f.source)+'/'),('PHYSICAL_DEMOS',f.rel(f.first)+'/'),
                ('CAMERA_RETAKE_DEMOS',f.rel(f.retakes)+'/'),('CAMERA_RETAKE_PLAN',f.rel(f.plan_path)),('CAMERA_RETAKE_TOTAL',2)):
            mocked=patch.object(dashboard,key,value);mocked.start();self.addCleanup(mocked.stop)
        self.progress=dashboard.Progress(f.root)

    def test_overlay_keeps_first_success_when_retaken_episode_fails(self):
        data=self.progress.physical_execution()
        self.assertEqual(data['counts']['success'],2)
        self.assertEqual(data['counts']['failed'],1)
        self.assertEqual(data['counts']['video_count'],3)
        self.assertEqual(data['camera_retakes']['saved'],2)
        self.assertEqual(data['camera_retakes']['stale'],0)
        row=next(r for r in data['entries'] if r['scene_id']==self.fixture.ids[0])
        self.assertEqual(row['execution_status'],'success')
        self.assertEqual(row['camera_retake']['execution_status'],'failed')
        self.assertNotEqual(row['video_url'],row['camera_retake']['video_url'])

    def test_changed_retake_result_hides_only_new_links(self):
        f=self.fixture;row=f.retake_entries[0]
        f.write(f.retakes/row['execution_result_path'],b'changed result')
        data=self.progress.physical_execution()
        self.assertEqual(data['counts']['success'],2)
        self.assertEqual(data['counts']['stale'],0)
        self.assertEqual(data['camera_retakes']['stale'],1)
        self.assertEqual(data['camera_retakes']['saved'],1)
        first=next(r for r in data['entries'] if r['scene_id']==f.ids[0])
        self.assertIsNotNone(first['video_url']);self.assertIsNone(first['camera_retake']['video_url'])

    def test_retake_first_pass_outcome_or_plan_binding_cannot_be_forged(self):
        f=self.fixture
        f.change_actual(physical_attempt_number=1)
        data=self.progress.physical_execution()
        self.assertEqual(data['camera_retakes']['stale'],1)
        self.assertEqual(data['counts']['success'],2)
        self.assertEqual(data['counts']['failed'],1)

    def test_http_retake_links_require_live_bound_results(self):
        f=self.fixture
        server=ThreadingHTTPServer(('127.0.0.1',0),dashboard.handler_for(self.progress))
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            data=self.progress.physical_execution();row=data['camera_retakes']['entries'][0]
            url='http://127.0.0.1:'+str(server.server_port)+row['video_url']
            with urllib.request.urlopen(urllib.request.Request(url,headers={'Range':'bytes=0-7'})) as response:
                self.assertEqual(response.status,206);self.assertEqual(len(response.read()),8)
            f.write(f.retakes/f.retake_entries[0]['execution_result_path'],b'changed')
            with self.assertRaises(urllib.error.HTTPError) as caught:urllib.request.urlopen(url)
            self.assertEqual(caught.exception.code,404);caught.exception.close()
        finally:
            server.shutdown();server.server_close();thread.join()


if __name__=='__main__': unittest.main()
