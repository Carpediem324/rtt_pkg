from setuptools import setup

package_name = 'rtt_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shh',
    maintainer_email='gusgkr0324@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client_node = rtt_pkg.client_node:main',
            'server_node = rtt_pkg.server_node:main',
            'sv = rtt_pkg.sv:main',
            'cl = rtt_pkg.cl:main',
            'plot = rtt_pkg.plotter:main',
        ],
    },
)
