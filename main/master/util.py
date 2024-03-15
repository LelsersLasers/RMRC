import os
import cv2


def read_last_line(f):
    # with open('filename.txt', 'rb') as f:
    try:  # catch OSError in case of a one line file 
        f.seek(-2, os.SEEK_END)
        while f.read(1) != b'\n':
            f.seek(-2, os.SEEK_CUR)
    except OSError:
        f.seek(0)
        
    last_line = f.readline().decode()
    return last_line

def pretty_print_dict(d):
    import pprint
    pp = pprint.PrettyPrinter(indent=4)
    pp.pprint(d)