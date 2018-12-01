from setuptools import setup

setup(name='runroader',
      version='0.1',
      description='Speed planning with optimization',
      url='http://https://github.com/yodahuang/runroader',
      license='MIT',
      packages=['runroader'],
      entry_points={
          'console_scripts': [
              'create-runroader-env=scripts.create_environment:main',
              'optimize-runroader=runroader.optimize:optimize',
              'visualize-runroader-env=runroader.environment:visualize_pickle'
          ]
      },
      install_requires=[
          'cvxpy',
          'dccp',
          'matplotlib',
          'numpy',
          'click'
      ],
      zip_safe=False)
