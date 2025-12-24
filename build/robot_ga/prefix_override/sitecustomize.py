import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/majiidkun/Majiid/KULIAH/ITB/Semester3/Algeo/Makalah/install/robot_ga'
