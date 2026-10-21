from pathlib import Path
from docx import Document
from docx.text.paragraph import Paragraph
from docx.table import Table
import shutil

ROOT = Path(__file__).resolve().parent
OLD = ROOT.parent / 'SafeBench-5090-Observer-Guide-20261019'
doc = Document(OLD / '5090_IsaacLab_Arena安全采集与只读评估部署说明.docx')
out = []
code_lines = []


def flush_code():
    if code_lines:
        text = '\n'.join(code_lines)
        lang = 'text'
        if text.lstrip().startswith('{'):
            lang = 'json'
        elif any(x in text for x in ['from safety_risk', 'online = Observation']):
            lang = 'python'
        elif any(x in text for x in ['ARENA_CONTAINER=', 'python -m unittest', 'docker inspect']):
            lang = 'bash'
        out.append('```' + lang + '\n' + text + '\n```')
        code_lines.clear()


for child in doc.element.body.iterchildren():
    if child.tag.endswith('}p'):
        para = Paragraph(child, doc)
        if para.style.name == 'Code Guide':
            code_lines.append(para.text.rstrip())
            continue
        flush_code()
        text = para.text.strip()
        if not text:
            continue
        if para.style.name == 'Title':
            text = '# ' + text.replace('\n', '：')
        elif para.style.name.startswith('Heading '):
            text = '#' * (int(para.style.name.split()[-1]) + 1) + ' ' + text
        out.append(text)
    elif child.tag.endswith('}tbl'):
        flush_code()
        table = Table(child, doc)
        rows = [[c.text.replace('|', '\\|').replace('\n', '<br>') for c in r.cells] for r in table.rows]
        out.append('\n'.join(['| ' + ' | '.join(rows[0]) + ' |', '| ' + ' | '.join(['---'] * len(rows[0])) + ' |'] + ['| ' + ' | '.join(r) + ' |' for r in rows[1:]]))
flush_code()
intro = '> 本 Markdown 版附有源端参考代码。先阅读 [参考代码使用说明](reference_code/README.md)，再按本文实施。代码路径与 SHA256 见 `reference_code/SOURCE_MANIFEST.json`。源代码是未修改快照；Arena 接入层仍需实现。'
out.insert(2, intro)
text = '\n\n'.join(out) + '\n'
text = text.replace('交付包括本说明、同内容纯文本、建议的新增配置文件、环境探测脚本和验收清单。', '本次交付包括 Markdown 说明、源端参考代码、建议的新增配置文件、环境探测脚本和验收清单。')
text = text.replace('模板默认采用 shadow，所有 observer 写动作权限为 false；采用 4 个环境，但试接入先将 pilot 环境数设置为 1。', '模板默认采用 shadow，所有 observer 写动作权限为 false；pilot.num_envs 已设为 1，production_reference.num_envs 为 4。')
(ROOT / '5090_IsaacLab_Arena安全采集与只读评估部署说明.md').write_text(text, encoding='utf-8')
dest = ROOT / 'reference_code' / 'arena_templates'
dest.mkdir(exist_ok=True)
for name in ['safety_observer_arena.json', 'probe_safety_observer_env.py']:
    shutil.copyfile(OLD / name, dest / name)
checklist = (OLD / '使用说明与验收清单.txt').read_text()
checklist = checklist[checklist.index('实施顺序'):checklist.index('完成边界')]
checklist = checklist.replace('实施顺序', '# 实施与验收清单', 1).replace('□ ', '- [ ] ').replace('必须保留的语义', '## 必须保留的语义')
(ROOT / 'reference_code' / '验收清单.md').write_text(checklist, encoding='utf-8')
print(ROOT / '5090_IsaacLab_Arena安全采集与只读评估部署说明.md')
