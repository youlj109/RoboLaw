# Local Scene Review

Open <http://127.0.0.1:8765> while the local service is running.

Latest [quality recheck](QUALITY_RECHECK_01.md): nine image-only decisions,
one glass-material revision and one new adapted-pregrasp query. New appearance
does not automatically inherit the old recordings' physical acceptance.

The page now separates image-only revision, approach planning and full physical
execution. The latter has actual videos, audit links and its own autosaved
notes. See [physical completion](PHYSICAL_COMPLETION_20261027.md) for the latest
bounded task acceptance; earlier approach paths do not count as task success.

This is a visual review of the nine original screenshots in
`scene_screenshots_20261026/`. It does not run the simulator or change layouts.
"Good" is a preliminary visual assessment, not a safety acceptance result.
The page provides thumbnails, a large image, full-image view, status filters,
progress, observations, adjustment plans, verification notes and user notes.

## Saving

Edits save after a 700 ms pause. Wait for the green saved-to-disk message.
The authoritative local store is `data/reviews.sqlite3`, with a history row for
each saved edit. Browser draft storage provides an additional recovery aid,
not a replacement for server persistence. Both drafts and edits stay local.
Polling every three seconds updates progress without replacing unsaved fields.
Concurrent edits to the same field are rejected until the user resolves them;
edits to different fields merge. The page never executes text entered as notes.
Use the export link for JSON containing current annotations and edit history.
The original nine image-only assessments are also captured in the generated
`data/initial_visual_review.json`; current user changes stay in the database.

The database is intentionally ignored by Git. Back it up while the service is
stopped, or use JSON export while it runs; do not copy only the main SQLite file
while there may be an active WAL journal. Images and simulator data are unchanged.

## Starting Again

From `/home/zxw/RoboLaw`:

```bash
python3 scripts/extensions/controls/scene_review_server.py
```

The currently running transient user service survives closing the terminal, but
not reboot. While it exists:

```bash
systemctl --user status robolaw-scene-review
systemctl --user stop robolaw-scene-review
systemctl --user start robolaw-scene-review
```

After reboot, either run the Python command above or recreate the service:

```bash
systemd-run --user --unit=robolaw-scene-review --property=Restart=on-failure /usr/bin/python3 /home/zxw/RoboLaw/scripts/extensions/controls/scene_review_server.py
```

The server binds only to loopback, serves a fixed allowlist of files, rejects
foreign Host/Origin values and uses no third-party frontend resources.

## Initial Visual Triage

| Scene | Preliminary verdict | Main point |
| --- | --- | --- |
| Left button / human | Good, retain main layout | Hand in shared workspace; add whole-body and target views |
| Left button / property | Good, retain main layout | Recognizable upright vase; verify support and stability |
| Left button / obstacle | Change presentation, then inspect layout | Support and appliance separation obscured |
| Right button / human | Good, retain main layout | Plausible reach; add right-target and whole-body views |
| Right button / property | Good, retain main layout | Recognizable vase near approach region |
| Right button / obstacle | Change presentation, then inspect layout | Target/support obscured; verify final press clearance |
| Oven / human | Good, retain main layout | Standing person and extended hand visible; verify path relevance |
| Oven / property | Change | Cylinder's property/fragility semantics unclear |
| Oven / obstacle | Good as a geometric control | Visible support frame; household realism optional |

These are image-only triage decisions. Occlusion is not proof of penetration,
and apparent proximity is not proof of a baseline violation. A requested image
improvement does not by itself require moving a physically validated layout.
Current editable annotations and further user decisions live in the database.

## Tests

```bash
python3 -m pytest -q tests/extensions/controls/test_scene_review_server.py
node --check docs/extensions/controls/scene_review/app.js
```

Validated on 2026-10-27: 8 storage/HTTP tests passed. An actual Firefox 136
WebDriver BiDi smoke test also passed desktop/mobile rendering, autosave,
refresh recovery, scene navigation, image zoom, filtering, conflict rejection,
draft recovery and explicit conflict resolution. Smoke edits used a separate
scratch database on port 8766; production annotations were not changed.
Generated desktop/mobile page captures are in `data/page_desktop.png` and
`data/page_mobile.png`. The browser smoke script is
`tests/extensions/controls/scene_review_browser_smoke.cjs`; it requires a fresh
scratch server on port 8766, a separate Firefox remote-debugging profile on
port 9223 and `PW_WS_MODULE` pointing to a Playwright utilsBundle module that
exports `ws`. These are test-only dependencies; the application uses only the
Python standard library and browser-native JavaScript.
