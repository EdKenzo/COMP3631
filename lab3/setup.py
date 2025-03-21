from setuptools import find_packages, setup

package_name = 'lab3'

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
    maintainer='Rafael Papallas',
    maintainer_email='r.papallas@leeds.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'firstwalk = lab3.firstwalk:main',
            'circlewalk = lab3.exercise1:main',
            'squarewalk = lab3.exercise2:main'
        ],
    },
)
