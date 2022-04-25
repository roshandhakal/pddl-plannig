from setuptools import setup, find_packages


setup(
    name="plan",
    version="1.0.0",
    description="Multi Agent Planning with PDDL",
    license="MIT",
    author="Roshan Dhakal",
    author_email="rdhakal2@gmu.edu",
    packages=find_packages(),
    package_data={"": ["*.pddl"]},
    install_requires=["numpy", "matplotlib"],
)
