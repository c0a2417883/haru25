from setuptools import find_packages, setup

package_name = 'haru25_tools'#パッケージ名

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nagasima',
    maintainer_email='nagasima@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temp = haru25_tools.temp:main', #add Temp Node　
            'robomas = haru25_tools.robomas:main',#GUI　プログラム名
            'robomas_control = haru25_tools.robomas_control:main',#usb to can経由　プログラム名
            'robomas_teensy = haru25_tools.robomas_teensy:main',#teensy経由　プログラム名
            'scan_conv = haru25_tools.scan_conv:main',#teensy経由　プログラム名
            'pos2tf = haru25_tools.pos2tf:main',#teensy経由　プログラム名
            'record = haru25_tools.record:main'#teensy経由　プログラム名
        ],
    },
)
