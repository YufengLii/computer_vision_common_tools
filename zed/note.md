-i https://pypi.douban.com/simple --trust -host=pypi.douban.com

- https://github.com/stereolabs/zed-python-api
- https://github.com/stereolabs/zed-examples
- https://support.stereolabs.com/hc/en-us/articles/207616785-Getting-Started-with-your-ZED-camera



1. download cuda and install
`shell
sudo chmod +x cuda_10.2.89_440.33.01_linux.run
sudo ./cuda_10.0......run
`

**多CDUA安装时都不要创建symbolic link**

Do you accept the previously read EULA? (accept/decline/quit): accept
You are attempting to install on an unsupported configuration. Do you wish to continue? ((y)es/(n)o) [ default is no ]: y
Install NVIDIA Accelerated Graphics Driver for Linux-x86_64 346.46? ((y)es/(n)o/(q)uit): n
Do you want to install the OpenGL libraries? ((y)es/(n)o/(q)uit) [ default is yes ]: n
Install the CUDA 10.0 Toolkit? ((y)es/(n)o/(q)uit): y
Enter Toolkit Location [ default is /usr/local/cuda-10.0 ]:
/usr/local/cuda-10.0 is not writable.
Do you wish to run the installation with ‘sudo’? ((y)es/(n)o): y
Please enter your password:
Do you want to install a symbolic link at /usr/local/cuda? ((y)es/(n)o/(q)uit): n
Install the CUDA 10.0 Samples? ((y)es/(n)o/(q)uit): y
Enter CUDA Samples Location [ default is /home/xxx ]:
Installing the CUDA Toolkit in /usr/local/cuda-10.0 …
Installing the CUDA Samples in /home/xxx …
Copying samples to /home/xxx/NVIDIA_CUDA-10.0_Samples now…
Finished copying samples.


2. download zed sdk

`shell
chmod u+x ZED_SDK_Ubuntu18_cuda10.2_v3.2.2.run
./ ZED_SDK_Ubuntu18_cuda10.2_v3.2.2.run


3.
virturalenv
 
python --version
pip3 install cython numpy
pip3 install opencv-python pyopengl


$ cd "/usr/local/zed/"
$ python get_python_api.py

    # The script displays the detected platform versions
    CUDA 10.0
    Platform ubuntu18
    ZED 3.1
    Python 3.7
    # Downloads the whl package
    Downloading python package from https://download.stereolabs.com/zedsdk/3.1/ubuntu18/cu100/py37 ...

    # Gives instruction on how to install the downloaded package
    File saved into pyzed-3.1-cp37-cp37m-linux_x86_64.whl
    To install it run :
      python3 -m pip install pyzed-3.1-cp37-cp37m-linux_x86_64.whl



4. git clone https://github.com/stereolabs/zed-examples















