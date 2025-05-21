from setuptools import find_packages, setup

package_name = 'av_technology'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sinan KARAKAYA',
    maintainer_email='karakaya.sinan@proton.me',
    description='This package contains the nodes for the finals exams of the AV Technology course from Korea University.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous = av_technology.autonomous:main',
            'qrcode_detector = av_technology.qrcode_detector:main',
            'traffic_light_detector = av_technology.traffic_light_detector:main'
        ],
    },
)
