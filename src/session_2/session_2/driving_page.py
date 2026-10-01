"""한 파일을 편집하고 저장하는 Python 코드 창."""
import ast
import html
import keyword
from pathlib import Path
import re
import sys
import textwrap

from PySide6.QtCore import Qt, QRect, QSize, QSaveFile, QIODevice
from PySide6.QtGui import (QColor, QFont, QFontDatabase, QPainter, QSyntaxHighlighter,
                          QTextCharFormat, QTextCursor, QKeySequence, QShortcut)
from PySide6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPlainTextEdit,
                              QPushButton, QLabel, QMessageBox, QLineEdit,
                              QSplitter, QTextBrowser, QStackedWidget)


LESSONS = [
    ('이미지 전처리 & 차선 검출\n(segmentation 모델)', '사진에서 도로와 차선 찾기',
     '사진의 각 점이 도로인지 차선인지 구분합니다.',
     [('크기 맞추기', '사진을 640 × 352 크기로 맞춥니다.'),
      ('색 값 정리하기', 'BGR을 RGB로 바꾸고, 숫자 범위를 학습 때와 같게 맞춥니다.'),
      ('모델에 넣기', '사진 배열 순서를 바꿔 PIDNet에 넣습니다.'),
      ('차선 번호 고르기', '각 점에서 점수가 가장 높은 번호를 고릅니다. 실선은 2, 점선은 3입니다.')],
     '사진 → 도로·차선 번호 배열',
     '차선용 PIDNet 모델입니다. 2번의 YOLOv8과는 다릅니다.'),
    ('주행 오차 계산', '차선과 목표 위치의 차이 구하기',
     '사진 속 차선 위치를 원하는 바퀴 각도로 바꿉니다.',
     [('차선 위치 찾기', '사진 아래쪽 75% 지점 주변 5줄에서 이어진 차선 점들을 찾습니다.'),
      ('기준 차선 고르기', '기본값은 오른쪽 실선입니다. 오른쪽 절반에서 가장 오른쪽 차선의 중심을 고릅니다.'),
      ('거리 차이 구하기', '오차 = 목표 위치 − 차선 위치. 목표 위치는 기본적으로 사진 너비의 79%입니다.'),
      ('목표 각도 정하기', '−오차 ÷ 130 × 45로 계산하고 −45~45도로 제한합니다.')],
     '차선 위치 → 가로 오차 → 목표 바퀴 각도',
     '차선을 못 찾으면 0이 아니라 None을 반환합니다.'),
    ('조향 안정화', '바퀴 움직임을 부드럽게 만들기',
     '목표 각도와 실제 각도의 차이로 조향 모터 힘을 계산합니다.',
     [('각도 차이 구하기', '오차 = 목표 각도 − 실제 각도입니다.'),
      ('P와 I 계산하기', 'P는 현재 오차 × 6.5입니다. I는 쌓인 오차를 반영하지만 기본 계수가 0이라 사용하지 않습니다.'),
      ('D로 흔들림 줄이기', '실제 바퀴 각속도 × −0.8을 더해 급한 움직임을 줄입니다.'),
      ('힘 제한하기', 'P + I + D를 −150~150으로 제한합니다. 오차 1도 이내는 0, 움직일 때 최소 힘의 크기는 40입니다.')],
     '목표·실제 각도와 각속도 → 조향 모터 힘',
     '앞 단계는 사진 속 거리 오차, 이 단계는 바퀴 각도 오차입니다.'),
    ('전체 코드', '사진부터 모터 명령까지 연결하기',
     '앞의 세 계산과 모터 보드 통신을 한 파일에서 봅니다.',
     [('차선 찾기', 'Segmenter가 사진을 도로·차선 번호로 바꿉니다.'),
      ('목표 각도 정하기', 'scan_line과 DrivingPipeline이 차선 위치로 돌릴 각도를 정합니다.'),
      ('조향 힘 계산하기', '보드에서 받은 실제 바퀴 각도와 목표 각도를 PID로 비교합니다.'),
      ('명령 보내기', 'MotorControl이 0.05초마다 힘을 보냅니다. 새 정보가 끊기면 멈춥니다.')],
     '사진 → 차선 → 목표 각도 → 조향 힘 → 모터 보드',
     '편집·저장은 이 화면에서 합니다. 코드를 자동 실행하지 않습니다.'),
]


def lesson_source(source, index):
    """현재 편집 내용에서 발췌한다. 문법 오류 때 과거 예제로 대체하지 않는다."""
    tree = ast.parse(source)
    names = [('Segmenter',), ('scan_line',), ('PID',)][index]
    pieces = []
    for node in tree.body:
        if getattr(node, 'name', None) in names:
            if index == 0:
                predict = next((n for n in node.body if isinstance(n, ast.FunctionDef) and n.name == 'predict'), None)
                if predict is None:
                    raise ValueError('Segmenter.predict 함수를 전체 코드에서 확인하세요.')
                lines = source.splitlines(keepends=True)[predict.lineno-1:predict.end_lineno]
                method = textwrap.dedent(''.join(lines)).rstrip()
                pieces.append('# 모델을 준비하는 __init__은 전체 코드에서 볼 수 있습니다.\nclass Segmenter:\n' + textwrap.indent(method, '    '))
            else:
                pieces.append(ast.get_source_segment(source, node))
    if len(pieces) != len(names):
        raise ValueError('필요한 함수나 클래스 이름이 변경되었습니다. 전체 코드에서 확인하세요.')
    pipeline = next((n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'DrivingPipeline'), None)
    if pipeline and index in (0, 1):
        process = next((n for n in pipeline.body if isinstance(n, ast.FunctionDef) and n.name == 'process'), None)
        if process:
            for node in process.body:
                selected = (isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'image' for t in node.targets)) if index == 0 else (
                    isinstance(node, ast.If) and 'scan.error_px' in ast.unparse(node.test))
                if selected:
                    lines = source.splitlines(keepends=True)[node.lineno-1:node.end_lineno]
                    excerpt = '# DrivingPipeline.process의 계산\n' + textwrap.dedent(''.join(lines)).rstrip()
                    if index == 0:
                        pieces.insert(0, excerpt)
                    else:
                        pieces.append(excerpt)
    return '\n\n'.join(pieces) + '\n'


class PythonColors(QSyntaxHighlighter):
    def __init__(self, document):
        super().__init__(document)
        self.colors = {}
        for name, color in [('keyword', '#c586c0'), ('string', '#ce9178'),
                            ('comment', '#6a9955'), ('number', '#b5cea8'),
                            ('function', '#dcdcaa')]:
            fmt = QTextCharFormat()
            fmt.setForeground(QColor(color))
            self.colors[name] = fmt
        self.tokens = re.compile(
            r'''(?P<comment>\#.*)|(?P<string>"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*')|(?P<number>\b\d+(?:\.\d+)?\b)|(?P<word>\b[A-Za-z_]\w*\b)''')

    def highlightBlock(self, text):
        self.setCurrentBlockState(0)
        offset = 0
        state = self.previousBlockState()
        if state in (1, 2):
            delimiter = "'''" if state == 1 else '"""'
            end = text.find(delimiter)
            if end < 0:
                self.setFormat(0, len(text), self.colors['string'])
                self.setCurrentBlockState(state)
                return
            offset = end + 3
            self.setFormat(0, offset, self.colors['string'])
        while offset < len(text):
            triple = re.search("'''|\"\"\"", text[offset:])
            limit = offset + triple.start() if triple else len(text)
            # A triple quote inside a comment or ordinary string is not a block start.
            for match in self.tokens.finditer(text, offset):
                if match.start() >= limit:
                    break
                if match.end() > limit:
                    limit = len(text)
                    triple = None
                kind = match.lastgroup
                if kind == 'word':
                    word = match.group()
                    kind = 'keyword' if word in keyword.kwlist else (
                        'function' if text[match.end():].lstrip().startswith('(') else None)
                if kind in self.colors:
                    self.setFormat(match.start(), match.end()-match.start(), self.colors[kind])
            if not triple:
                return
            delimiter = text[limit:limit+3]
            end = text.find(delimiter, limit+3)
            if end < 0:
                self.setFormat(limit, len(text)-limit, self.colors['string'])
                self.setCurrentBlockState(1 if delimiter == "'''" else 2)
                return
            self.setFormat(limit, end+3-limit, self.colors['string'])
            offset = end+3


class LineNumbers(QWidget):
    def __init__(self, editor):
        super().__init__(editor)
        self.editor = editor

    def sizeHint(self):
        return QSize(self.editor.number_width(), 0)

    def paintEvent(self, event):
        self.editor.paint_numbers(event)


class CodeEditor(QPlainTextEdit):
    def __init__(self):
        super().__init__()
        self.setLineWrapMode(QPlainTextEdit.NoWrap)
        families = QFontDatabase.families()
        family = next((name for name in ('Menlo', 'Consolas', 'DejaVu Sans Mono', 'Courier New') if name in families), QApplication.font().family())
        self.setFont(QFont(family))
        self.setStyleSheet('QPlainTextEdit {background:#1e1e1e;color:#d4d4d4;'
                          f'font-family:"{family}";font-size:14px;border:0;'
                          'border-radius:0;padding:4px;selection-background-color:#264f78;}')
        self.setTabStopDistance(self.fontMetrics().horizontalAdvance(' ') * 4)
        self.numbers = LineNumbers(self)
        self.blockCountChanged.connect(self.update_margin)
        self.updateRequest.connect(self.update_numbers)
        self.cursorPositionChanged.connect(self.numbers.update)
        self.highlighter = PythonColors(self.document())
        self.update_margin()

    def number_width(self):
        return 20 + self.fontMetrics().horizontalAdvance('9') * len(str(max(1, self.blockCount())))

    def update_margin(self, *_):
        self.setViewportMargins(self.number_width(), 0, 0, 0)

    def update_numbers(self, rect, dy):
        if dy:
            self.numbers.scroll(0, dy)
        else:
            self.numbers.update(0, rect.y(), self.numbers.width(), rect.height())
        if rect.contains(self.viewport().rect()):
            self.update_margin()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        rect = self.contentsRect()
        self.numbers.setGeometry(QRect(rect.left(), rect.top(), self.number_width(), rect.height()))

    def paint_numbers(self, event):
        painter = QPainter(self.numbers)
        painter.fillRect(event.rect(), QColor('#1e1e1e'))
        painter.setFont(self.font())
        block = self.firstVisibleBlock()
        top = round(self.blockBoundingGeometry(block).translated(self.contentOffset()).top())
        while block.isValid() and top <= event.rect().bottom():
            height = round(self.blockBoundingRect(block).height())
            if block.isVisible() and top + height >= event.rect().top():
                active = block.blockNumber() == self.textCursor().blockNumber()
                painter.setPen(QColor('#cccccc' if active else '#858585'))
                painter.drawText(0, top, self.numbers.width()-10, self.fontMetrics().height(),
                                 Qt.AlignRight, str(block.blockNumber()+1))
            top += height
            block = block.next()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Tab and not event.modifiers():
            self.insertPlainText('    ')
        elif event.key() in (Qt.Key_Return, Qt.Key_Enter) and not event.modifiers():
            cursor = self.textCursor()
            before = cursor.block().text()[:cursor.positionInBlock()]
            indent = re.match(r'\s*', before).group()
            if before.rstrip().endswith(':'):
                indent += '    '
            cursor.insertText('\n' + indent)
        else:
            super().keyPressEvent(event)


class DrivingPage(QWidget):
    def __init__(self, path):
        super().__init__()
        self.path = Path(path)
        layout = QVBoxLayout(self)
        tabs = QHBoxLayout()
        self.lesson_buttons = []
        for index, lesson in enumerate(LESSONS):
            button = QPushButton(lesson[0].replace('&', '&&'))
            button.setMinimumHeight(58)
            button.setCheckable(True)
            button.toggled.connect(lambda checked, i=index: self.select_lesson(i) if checked else None)
            tabs.addWidget(button, 1)
            self.lesson_buttons.append(button)
        layout.addLayout(tabs)
        split = QSplitter(Qt.Horizontal)
        self.explanation = QTextBrowser()
        self.explanation.setOpenExternalLinks(False)
        self.explanation.setMinimumWidth(220)
        self.explanation.setStyleSheet('QTextBrowser {background:white;color:#18304b;border:1px solid #c3cede;border-radius:8px;padding:16px;}')
        self.explanation.setAccessibleName('코드의 역할과 계산 과정')
        split.addWidget(self.explanation)
        right = QWidget()
        code_layout = QVBoxLayout(right)
        code_layout.setContentsMargins(0, 0, 0, 0)
        right.setMinimumWidth(320)
        row = QHBoxLayout()
        self.filename = QLabel(self.path.name)
        self.filename.setToolTip(str(self.path))
        row.addWidget(self.filename, 1)
        self.save_button = QPushButton('저장 · ⌘S / Ctrl+S')
        self.save_button.clicked.connect(self.save)
        row.addWidget(self.save_button)
        code_layout.addLayout(row)
        self.search = QLineEdit()
        self.search.setPlaceholderText('코드에서 찾기 · Enter: 다음 결과 · Esc: 닫기')
        self.search.returnPressed.connect(self.find_next)
        self.search.hide()
        code_layout.addWidget(self.search)
        self.code = CodeEditor()
        template = Path(__file__).with_name('autonomous_driving.py')
        self.code.setPlainText((self.path if self.path.exists() else template).read_text(encoding='utf-8'))
        self.code.document().setModified(False)
        self.code.document().modificationChanged.connect(self.modified)
        self.preview = CodeEditor()
        self.preview.setReadOnly(True)
        self.preview.setAccessibleName('선택한 부분 코드')
        self.code.setAccessibleName('전체 코드 편집기')
        self.code_stack = QStackedWidget()
        self.code_stack.addWidget(self.preview)
        self.code_stack.addWidget(self.code)
        code_layout.addWidget(self.code_stack, 1)
        split.addWidget(right)
        split.setChildrenCollapsible(False)
        split.setStretchFactor(0, 1)
        split.setStretchFactor(1, 2)
        split.setSizes([360, 760])
        layout.addWidget(split, 1)
        self.status = QLabel('차선 찾기 → 방향 정하기 → 핸들 조절')
        layout.addWidget(self.status)
        self.select_lesson(0)
        shortcuts = set()
        extra_modifier = 'Meta' if sys.platform == 'darwin' else 'Ctrl'
        for key, action in [(QKeySequence.Save, self.save), (QKeySequence.Find, self.open_search),
                            (extra_modifier+'+S', self.save), (extra_modifier+'+F', self.open_search), ('Escape', self.search.hide)]:
            sequence = QKeySequence(key)
            if sequence.toString() in shortcuts:
                continue
            shortcuts.add(sequence.toString())
            shortcut = QShortcut(sequence, self)
            shortcut.setContext(Qt.WidgetWithChildrenShortcut)
            shortcut.activated.connect(action)

    def select_lesson(self, index):
        self.lesson_index = index
        for i, button in enumerate(self.lesson_buttons):
            button.blockSignals(True)
            button.setChecked(i == index)
            button.blockSignals(False)
        _, title, role, steps, result, note = LESSONS[index]
        body = '<h2>' + html.escape(title) + '</h2><p style="color:#64748b">수업 기본 코드 기준 설명</p><h3>이 코드의 역할</h3><p>' + html.escape(role) + '</p><h3>계산 순서</h3>'
        for number, (heading, description) in enumerate(steps, 1):
            body += '<h4>' + str(number) + '. ' + html.escape(heading) + '</h4><p>' + html.escape(description) + '</p>'
        body += '<h3>입력과 결과</h3><p>' + html.escape(result) + '</p><p style="color:#64748b">' + html.escape(note) + '</p>'
        self.explanation.setHtml(body)
        self.code_stack.setCurrentIndex(1 if index == 3 else 0)
        self.save_button.setVisible(index == 3)
        self.search.hide()
        self.status.setText('차선 찾기 → 방향 정하기 → 핸들 조절' if index == 3 else '부분 코드 보기 · 수정은 전체 코드에서')
        if index != 3:
            try:
                self.preview.setPlainText(lesson_source(self.code.toPlainText(), index))
            except (SyntaxError, ValueError) as exc:
                self.preview.setPlainText('# 현재 전체 코드에서 이 부분을 읽을 수 없습니다.\n# 전체 코드 버튼을 눌러 확인하세요.\n')
                self.status.setText('부분 코드 표시 불가 · ' + str(exc))

    def active_editor(self):
        return self.code if self.lesson_index == 3 else self.preview

    def modified(self, changed):
        self.filename.setText(self.path.name + (' ● 수정됨' if changed else ''))

    def open_search(self):
        self.search.show()
        self.search.setFocus()
        self.search.selectAll()

    def find_next(self):
        if not self.search.text():
            return
        editor = self.active_editor()
        if not editor.find(self.search.text()):
            editor.moveCursor(QTextCursor.Start)
            if not editor.find(self.search.text()):
                self.status.setText('일치하는 글자가 없습니다.')

    def save(self):
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            output = QSaveFile(str(self.path))
            data = self.code.toPlainText().encode('utf-8')
            if not output.open(QIODevice.WriteOnly):
                raise OSError(output.errorString())
            if output.write(data) != len(data) or not output.commit():
                raise OSError(output.errorString())
        except OSError as exc:
            self.status.setText('저장 실패: ' + str(exc))
            return False
        self.code.document().setModified(False)
        self.status.setText('저장 완료 · ' + str(self.path))
        return True

    def confirm_close(self):
        if not self.code.document().isModified():
            return True
        answer = QMessageBox.question(self, '코드 저장', '수정한 코드를 저장할까요?',
            QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel, QMessageBox.Save)
        if answer == QMessageBox.Save:
            return self.save()
        if answer == QMessageBox.Discard:
            self.code.document().setModified(False)
            return True
        return False
