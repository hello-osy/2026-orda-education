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
    ('이미지 전처리 & 차선 검출\n(segmentation 모델)', '이미지 전처리 & 차선 검출',
     'Segmentation 모델로 사진의 각 픽셀이 어떤 클래스에 속하는지 판단합니다.',
     [('이미지 크기 변경', 'Segmentation 모델에 호환되도록 카메라 이미지를 640 × 352 크기로 변경합니다.'),
      ('BGR → RGB 변경', 'Segmentation 모델에 호환되도록 기존 BGR 순서로 저장된 카메라 이미지 데이터를 RGB 순서로 변경하고, 학습 때와 같은 값의 범위로 정규화합니다.'),
      ('사진 데이터 구조 변경', 'Segmentation 모델에 호환되도록 [세로, 가로, 색상] 배열을 [사진 수, 색상, 세로, 가로] 구조로 변경합니다.'),
      ('Segmentation 모델 추론', '각 픽셀이 어떤 클래스에 속하는지 나타내는 클래스별 점수 벡터가 나옵니다.'),
      ('픽셀별 클래스 결정', '각 픽셀에서 점수가 가장 높은 클래스를 선택합니다. 확률로 변환해도 가장 높은 클래스는 같습니다. 실선은 2, 점선은 3입니다.')],
     ''),
    ('주행 오차 계산', '주행 오차 계산',
     'Scan line으로 오른쪽 차선을 찾고, 기준 위치와의 차이를 조향값으로 변환합니다.',
     [('차선 픽셀 덩어리 찾기', '사진 아래쪽 75% 지점 주변 5줄에서 연속된 차선 픽셀 덩어리를 찾습니다.'),
      ('오른쪽 차선만 남기기', '사진의 오른쪽 부분에서 검출된 차선 픽셀 덩어리만 남기고, 왼쪽에서 검출된 것은 버립니다. 남은 실선 덩어리 중 가장 오른쪽 덩어리의 중심을 관측값으로 사용합니다.'),
      ('주행 오차 계산', '주행 오차 = 기준값 − 관측값입니다. 기준값은 정상 주행일 때의 카메라 화면을 고려해 우리가 정합니다. 기본 코드는 사진 너비의 약 79% 위치를 사용합니다.'),
      ('조향값으로 변환', '주행 오차를 −45~45도의 조향값 범위로 변환합니다. 기본 코드는 −오차 ÷ 130 × 45로 계산한 뒤 이 범위로 제한합니다.')],
     ''),
    ('조향 안정화', '조향 안정화',
     '목표 각도와 실제 각도의 차이를 이용해 바퀴 움직임을 부드럽게 만듭니다.',
     [('P값과 D값 튜닝', '기본 코드에서는 I값이 0이므로, 사람이 P값과 D값을 튜닝하면 됩니다. P는 각도 오차에 따른 조향 힘을 정하고, D는 바퀴의 급한 움직임을 줄입니다.'),
      ('좌우 흔들림 조정', '차가 좌우로 흔들리면서 주행하면 P값 또는 D값을 조정합니다.'),
      ('속도에 맞게 다시 튜닝', '속도가 바뀔 때마다 그 상황에 맞는 P값과 D값을 찾아야 합니다.')],
     ''),
    ('전체 코드', '인지 → 판단 → 제어',
     '자율주행의 전체 흐름을 인지, 판단, 제어 단계로 살펴봅니다.',
     [('인지', '센서 입력 받기 (ROS2)'),
      ('판단', '2-1. 이미지 전처리 & 차선 검출: segmentation 모델\n2-2. 주행 오차 계산: scan line\n2-3. 조향 안정화: P·D 튜닝'),
      ('제어', '모터 제어 명령 (ROS2)')],
     '위 흐름은 ROS2 기반 자율주행의 구성입니다. 이 교육용 파일의 모터 통신은 시리얼 방식입니다. 편집·저장은 이 화면에서 하며 코드를 자동 실행하지 않습니다.'),
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
        self.status = QLabel('인지 → 판단 → 제어')
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
        _, title, role, steps, note = LESSONS[index]
        body = '<h2>' + html.escape(title) + '</h2><p style="color:#64748b">수업 기본 코드 기준 설명</p><h3>이 코드의 역할</h3><p>' + html.escape(role) + '</p><h3>계산 순서</h3>'
        for number, (heading, description) in enumerate(steps, 1):
            body += '<h4>' + str(number) + '. ' + html.escape(heading) + '</h4><p>' + html.escape(description).replace('\n', '<br>') + '</p>'
        if note:
            body += '<p style="color:#64748b">' + html.escape(note) + '</p>'
        self.explanation.setHtml(body)
        self.code_stack.setCurrentIndex(1 if index == 3 else 0)
        self.save_button.setVisible(index == 3)
        self.search.hide()
        self.status.setText('인지 → 판단 → 제어' if index == 3 else '부분 코드 보기 · 수정은 전체 코드에서')
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
