from setuptools import find_packages, setup

package_name = 'calibration_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'calibration_pkg.camera_calibration': ['*.yaml'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cjh',
    maintainer_email='jchenjb@connect.ust.hk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_collection = calibration_pkg.hand_eye_calibration.data_collection:main',
            'hand_eye_calibration = calibration_pkg.hand_eye_calibration.hand_eye_calibration:main',
            'calibration_evaluation = calibration_pkg.hand_eye_calibration.calibration_evaluation:main',
        ],
    },
)
