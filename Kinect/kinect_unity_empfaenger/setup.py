from setuptools import setup

package_name = 'kinect_unity_empfaenger'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DEIN NAME',
    maintainer_email='deine@email.de',
    description='Empfänger für Kinect-Daten aus Unity',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kin_empfaenger = kinect_unity_empfaenger.kin_empfaenger:main'
        ],
    },
)
