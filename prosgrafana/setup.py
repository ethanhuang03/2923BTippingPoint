# setup.py for non-frozen builds

from setuptools import setup, find_packages

with open('requirements.txt') as f:
    required = f.read().splitlines()

setup(
    name='pros-grafana-cli',
    version=open('pip_version').read().strip(),
    packages=find_packages(),
    url='https://github.com/purduesigbots/pros-grafana-cli',
    license='MPL-2.0',
    author='Yerti',
    # author_email='',
    description='Command Line Interface for interacting with Grafana through a V5 Brain.',
    install_requires=required,
    entry_points={
        'console_scripts': [
            'prosgrafana=prosgrafana.cli.gui:gui',
        ]
    }
)
