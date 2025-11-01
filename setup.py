from setuptools import find_packages, setup

package_name = 'hand_palm_ur10'

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
    maintainer='trungtran',
    maintainer_email='trungtran@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'palm_detector = hand_detector.palm_hand_publish:main',
            'palm_controller = hand_detector.palm_to_script:main', 
        ],
    },
)
