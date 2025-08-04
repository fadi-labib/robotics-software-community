from setuptools import setup

package_name = 'temperature_broadcaster'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nathan',
    maintainer_email='nathan@example.com',
    description='Test broadcaster node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_broadcaster_node = temperature_broadcaster.test_broadcaster_node:main',
        ],
    },
)
