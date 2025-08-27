from zipfile import ZipFile
from sys import argv

with ZipFile(argv[1], 'r') as f:
    f.extractall()