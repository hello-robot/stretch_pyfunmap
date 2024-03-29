import setuptools
from Cython.Build import cythonize
from stretch_pyfunmap.version import __version__

with open("../README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="hello_robot_stretch_pyfunmap",
    version=__version__,
    author="Hello Robot Inc.",
    author_email="support@hello-robot.com",
    description="Stretch FUNMAP high level Python API",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/hello-robot/stretch_pyfunmap",
    packages=setuptools.find_packages(),
    ext_modules=cythonize("stretch_pyfunmap/cython_min_cost_path.pyx"),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
        "License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)"
    ],
    install_requires=[
        'numpy', 'scikit-image', 'Cython', 'numba', 'urchin'
    ]
)
