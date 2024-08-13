import subprocess
import sys

def install_packages(packages):
    for package in packages:
        print(f"Installing {package}...")
        try:
            result = subprocess.run([sys.executable, "-m", "pip", "install", package], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            print(result.stdout)
            if result.stderr:
                print(result.stderr)
        except subprocess.CalledProcessError as e:
            print(f"Failed to install {package}: {e}")
            sys.exit(1)

if __name__ == "__main__":
    packages = ['vpython', 'numpy', 'typing', 'pyqtgraph', 'pyqt5' , 'mcculw']
    source_dir = "./mcculw_files"  # Relative path to the local directory
    dest_dir = "C:/absolute/path/to/directory"  # Absolute path to the destination directory
    
    install_packages(packages)
    
    print("Installation and file copy complete.")
