from setuptools import find_packages, setup

package_name = 'Kamerabild'

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
    maintainer='benutzer',
    maintainer_email='benutzer@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'kamerabild1 = Kamerabild.kamerabild1:main',
        'lidar1 = Kamerabild.lidar1:main',
        'testScan = Kamerabild.testScan:main'
        ],
    },
)
