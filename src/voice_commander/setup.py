from setuptools import find_packages, setup

package_name = 'voice_commander'

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
    maintainer='lope626',
    maintainer_email='lope626@todo.todo',
    description='See README.md',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'voice_commander_node = voice_commander.voice_commander_node:main',
        ],
    },
)
