from setuptools import setup, find_packages

setup(
    name='state_machine',
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=[],
    entry_points={
        'console_scripts': [
            'main = state_machine.scripts.main:main',
        ],
    },
)
