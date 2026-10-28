"""Local annotation storage and HTTP contract; independent of Isaac/GPU."""

import importlib.util
import json
import threading
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

import pytest


SOURCE = Path(__file__).resolve().parents[3] / "scripts/extensions/controls/scene_review_server.py"
SPEC = importlib.util.spec_from_file_location("scene_review_server", SOURCE)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def test_seed_and_reopen(tmp_path):
    db = tmp_path / "review.sqlite3"
    store = module.Store(db)
    rows = store.snapshot()["scenes"]
    assert len(rows) == 9
    assert len({s["id"] for s in rows}) == 9
    sid = rows[0]["id"]
    store.patch(sid, {"changes": {"notes": "用户的调整方案"}, "base": {"notes": ""}})
    restored = module.Store(db).snapshot(history=True)
    assert restored["scenes"][0]["notes"] == "用户的调整方案"
    assert len(restored["history"]) == 1


def test_merge_fields_and_refuse_stale_overwrite(tmp_path):
    store = module.Store(tmp_path / "review.sqlite3")
    sid = store.snapshot()["scenes"][0]["id"]
    store.patch(sid, {"changes": {"plan": "A"}, "base": {"plan": ""}})
    store.patch(sid, {"changes": {"notes": "B"}, "base": {"notes": ""}})
    with pytest.raises(module.Conflict):
        store.patch(sid, {"changes": {"plan": "C"}, "base": {"plan": ""}})
    current = store.snapshot(history=True)
    assert current["scenes"][0]["plan"] == "A"
    assert current["scenes"][0]["notes"] == "B"
    assert len(current["history"]) == 2


def test_visual_review_is_separate_from_initial_and_user_notes(tmp_path):
    store = module.Store(tmp_path / "review.sqlite3")
    sid = store.snapshot()["scenes"][0]["id"]
    store.patch(sid, {"changes": {"notes": "keep me"}, "base": {"notes": ""}})
    result = store.patch(sid, {"changes": {"visual_status": "passed", "visual_images": [
        {"url": "/renders/scene/r01/wide.jpg", "label": "r01"}]},
        "base": {"visual_status": "pending", "visual_images": []}})
    assert result["status"] == "pending"
    assert result["notes"] == "keep me"
    assert result["visual_status"] == "passed"


def test_planning_updates_preserve_manual_annotation(tmp_path):
    store = module.Store(tmp_path / "review.sqlite3")
    sid = store.snapshot()["scenes"][0]["id"]
    store.patch(sid, {"changes": {"planning_notes": "try above bar", "planning_review": "change_waypoint"},
                      "base": {"planning_notes": "", "planning_review": "unreviewed"}})
    result = store.patch(sid, {"changes": {"planning_status": "failed", "planning_hint": "single query failed"},
                              "base": {"planning_status": "pending", "planning_hint": ""}})
    assert result["planning_notes"] == "try above bar"
    assert result["planning_review"] == "change_waypoint"
    assert result["visual_status"] == "pending"


def test_execution_is_separate_from_planning_and_preserves_user_notes(tmp_path):
    store = module.Store(tmp_path / 'review.sqlite3')
    row = store.snapshot()['scenes'][0]
    store.patch(row['id'], dict(changes={'planning_status': 'generated', 'execution_notes': 'keep me'},
        base={'planning_status': 'pending', 'execution_notes': ''}))
    updated = store.patch(row['id'], dict(changes={'execution_status': 'failed', 'execution_artifacts': [
        {'url': '/execution-data/round02/scene/all_cameras.mp4', 'label': 'Actual attempt'}]},
        base={'execution_status': 'pending', 'execution_artifacts': []}))
    assert updated['planning_status'] == 'generated'
    assert updated['execution_status'] == 'failed'
    assert updated['execution_notes'] == 'keep me'
    restored = module.Store(tmp_path / 'review.sqlite3').snapshot()['scenes'][0]
    assert restored['execution_artifacts'] == updated['execution_artifacts']


def test_complete_execution_log_can_exceed_old_annotation_limit(tmp_path):
    store = module.Store(tmp_path / 'review.sqlite3')
    row = store.snapshot()['scenes'][0]
    result = store.patch(row['id'], dict(changes={'execution_log':'x'*50000}, base={'execution_log':''}))
    assert len(result['execution_log']) == 50000
    with pytest.raises(ValueError):
        store.patch(row['id'], dict(changes={'execution_log':'x'*100001}, base={'execution_log':result['execution_log']}))


@pytest.mark.parametrize('field,value', [
    ('execution_status', 'automatically_safe'),
    ('execution_artifacts', [{'url': '/execution-data/../private.json', 'label': 'bad'}]),
    ('execution_artifacts', [{'url': 'https://example.org/video.mp4', 'label': 'bad'}]),
    ('execution_artifacts', [{'url': '/execution-data/script.py', 'label': 'bad'}]),
])
def test_execution_fields_reject_unsafe_values(tmp_path, field, value):
    store = module.Store(tmp_path / 'review.sqlite3')
    row = store.snapshot()['scenes'][0]
    with pytest.raises(ValueError):
        store.patch(row['id'], dict(changes={field: value}, base={field: row[field]}))


@pytest.mark.parametrize("field,value", [
    ("planning_status", "safe"), ("planning_review", "safety_passed"),
    ("planning_artifacts", [{"url": "/plans/../secret.json", "label": "bad"}]),
    ("planning_artifacts", [{"url": "https://evil.example/preview.json", "label": "bad"}]),
])
def test_planning_fields_reject_invalid_values(tmp_path, field, value):
    store = module.Store(tmp_path / "review.sqlite3")
    row = store.snapshot()["scenes"][0]
    with pytest.raises(ValueError):
        store.patch(row['id'], {"changes": {field: value}, "base": {field: row[field]}})


@pytest.mark.parametrize("value", [
    [{"url": "/renders/../private.jpg", "label": "bad"}],
    [{"url": "https://evil.example/a.jpg", "label": "bad"}],
    [{"url": "/renders/config.json", "label": "bad"}],
    "not an array",
])
def test_visual_references_reject_unsafe_paths(tmp_path, value):
    store = module.Store(tmp_path / "review.sqlite3")
    sid = store.snapshot()["scenes"][0]["id"]
    with pytest.raises(ValueError):
        store.patch(sid, {"changes": {"visual_images": value}, "base": {"visual_images": []}})


@pytest.mark.parametrize("changes,base", [
    ({"status": "passed"}, {"status": "pending"}),
    ({"image": "/etc/passwd"}, {"image": ""}),
    ({"plan": 12}, {"plan": ""}),
    ({"plan": "a" * 30001}, {"plan": ""}),
    ({"notes": "x"}, {}),
])
def test_invalid_edits_rejected(tmp_path, changes, base):
    store = module.Store(tmp_path / "review.sqlite3")
    sid = store.snapshot()["scenes"][0]["id"]
    with pytest.raises(ValueError):
        store.patch(sid, dict(changes=changes, base=base))
    assert not store.snapshot(history=True)["history"]


def test_http_routes_and_csrf(tmp_path):
    store = module.Store(tmp_path / "review.sqlite3")
    server = module.ThreadingHTTPServer(("127.0.0.1", 0), module.handler_for(store, tmp_path, 0))
    port = server.server_address[1]
    server.RequestHandlerClass = module.handler_for(store, tmp_path, port)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    client = build_opener(ProxyHandler({}))
    root = f"http://127.0.0.1:{port}"
    try:
        with client.open(root + "/") as response:
            assert b'lang="zh-CN"' in response.read()
            assert "frame-ancestors 'none'" in response.headers["Content-Security-Policy"]
        with client.open(root + "/benchmark.js") as response:
            assert b'renderBenchmarkSummary' in response.read()
            assert 'javascript' in response.headers['Content-Type']
        data = json.load(client.open(root + "/api/scenes"))
        sid = data["scenes"][0]["id"]
        body = json.dumps(dict(changes={"notes": "saved"}, base={"notes": ""})).encode()
        req = Request(root + "/api/scenes/" + sid, body, method="PATCH")
        assert json.load(client.open(req))["notes"] == "saved"
        with pytest.raises(HTTPError) as error:
            client.open(req)
            body2 = json.dumps(dict(changes={"notes": "stale"}, base={"notes": ""})).encode()
            client.open(Request(root + "/api/scenes/" + sid, body2, method="PATCH"))
        assert error.value.code == 409
        req = Request(root + "/api/scenes/" + sid, body, headers={"Origin": "https://evil.example"}, method="PATCH")
        with pytest.raises(HTTPError) as error:
            client.open(req)
        assert error.value.code == 403
        for path in ("/../scene_review/data/reviews.sqlite3", "/images/../../etc/passwd", "/data/reviews.sqlite3"):
            with pytest.raises(HTTPError) as error:
                client.open(root + path)
            assert error.value.code == 404
        assert len(json.load(client.open(root + "/api/export"))["history"]) == 2
    finally:
        server.shutdown()
        thread.join()
        server.server_close()
