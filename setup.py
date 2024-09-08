from setuptools import find_packages, setup

package_name = 'usi_angry_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='nandini.khandelwal.btech2021@sitpune.edu.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'angry_turtle_controller = usi_angry_turtle.angry_turtle_controller:main'
        ],
    },
)

