from setuptools import setup, find_packages

version = "0.0.2"

with open('README.md') as f:
    readme = f.read()

with open('LICENSE') as f:
    license = f.read()

extra_requirements = [
    "hidapi",
    "numpy>=1.17.4",
    "hid==1.0.4",
    "pyglm",
    "scipy",
    "ansitable",
    "progress",
    "typing_extensions",
    "ipykernel",
    "matplotlib"
]

install_requires = extra_requirements

setup(
    name='joycon-robotics',
    version=version,
    description='Nintendo Switch Joy-Con for robotics',
    long_description=readme,
    long_description_content_type='text/markdown',
    author='boxjod, Huanxu Lin',
    author_email=', '.join([
        'boxjod@163.com',
        "linhxforeduct@outlook.com"
    ]),
    url='https://github.com/box2ai-robotics/joycon-robotics',
    license=license,
    packages=find_packages(),
    install_requires=install_requires,  # 使用合并后的依赖列表
    classifiers=[
        'Programming Language :: Python :: 3.7'
    ]
)

