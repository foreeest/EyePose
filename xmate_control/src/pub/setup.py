from setuptools import setup

package_name = 'pub'
submodules = "pub/utils" # is added

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name ,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    # py_modules=['utils'],  # is added
    zip_safe=True,
    maintainer='gamemaster',
    maintainer_email='gamemaster@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'control_pub = pub.control_pub:main'
        ],
    },
)
