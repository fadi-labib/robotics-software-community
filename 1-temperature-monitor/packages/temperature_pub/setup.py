from setuptools import find_packages, setup

package_name = 'temperature_pub'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Youhana Beshay',
    maintainer_email='youhanabeshay@gmail.com',
    description='Package containing a publisher that simulates temperature readings from sensors',
    license='MPL-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "temp_pub = temperature_pub.temperature_pub:main"
        ],
    },
)
