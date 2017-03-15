
#simple python script that runs clang-format on the entire code base.
import subprocess
import os

def handleDirectory(dir):
  for root, dirs, files in os.walk(dir):
    for file in files:
      if file.endswith('.h') or file.endswith('.cpp'):
        subprocess.run([
          "clang-format",
          "-style=file" ,
          '-i' ,
          os.path.join(root, file)])

handleDirectory('include')
handleDirectory('src')
handleDirectory('tests')