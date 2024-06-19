import os

base_dir = os.path.dirname(os.path.abspath(__file__))

print(base_dir)

file_path = os.path.join(base_dir, '..', 'tmp', 'mtx.csv')

print(file_path)
