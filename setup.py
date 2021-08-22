from setuptools import setup


with open("README.md", "r") as fh:
    long_description = fh.read()

with open("requirements.txt", encoding="utf-8-sig") as f:
    requirements = f.readlines()

with open("LICENSE", encoding="utf-8-sig") as f:
    license = f.readlines()

setup(
    name='smart_mobility_utilities',
    version='0.0.1',
    author='Alaa Khamis and Yinan Wang',
    long_description=long_description,
    install_requires=requirements,
    license=license,
    python_requires=">=3.6"
)