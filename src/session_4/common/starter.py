"""학생의 student/driving.py 하나를 읽어 네 역할을 연결한다."""
from .student_runner import load_student, student_source

_student = load_student()
lane = _student.lane
rubbercone = _student.rubbercone
yolo = _student.yolo
DrivingController = _student.DrivingController
