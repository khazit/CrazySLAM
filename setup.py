from setuptools import setup

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name='CrazySLAM',
    version='0.1.0',
    author='Ali Benkassou',
    author_email='a.benkassou1@gmail.com',
    description="SLAM based localization on Crazyflie",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/khazit/CrazySLAM",
    packages=setuptools.find_packages(),
    classifiers=[
        "Operating System :: OS Independent",
    ],
    install_requires=['numpy', 'scikit-learn', 'matplotlib'],
    python_requires='>=3.6',
)
