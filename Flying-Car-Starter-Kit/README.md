# FlyingCar-Starter-Kit

Starter kit for Flying Car. Based on Python 3.

Packages used:

* [`matplotlib`](https://matplotlib.org/)
* [`jupyter`](http://jupyter.org/)
* [`udacidrone`](https://github.com/udacity/udacidrone). Due to `udacidrone` being updated as the ND progresses, it's recommended for new projects you update udacidrone with `pip install --upgrade udacidrone`.
* [`visdom`](https://github.com/facebookresearch/visdom/)


The recommended way to get up and running:

## Environment Setup

Step 1: Download miniconda from [here](https://conda.io/en/master/miniconda.html)

Step 2: You'll have to manually add miniconda to the $PATH using 

  ```
  export PATH="/path/to/miniconda/bin:$PATH"
  ```

Step 3: Create a conda env using,

  ```
  conda env create -f environment.yml
  ```

Note: If you run into any errors during the creation of the conda environment it might be likely that it's an
xcode setup issue.

First, clean up the stale environment build using,

```
conda env remove -n fc
```

and then rebuild it after 
```
xcode-select --install
```

Check if the environment is created using, you should be able to see `fc` environment in the list

```
conda info --envs
```

To clean up all the tarballs, run the following: 
```
conda clean -tp
```

Step 4: Now active the conda environment,

```
cd Flying-Car-Starter-Kit
conda activate fc
```

This should activate the `fc` env, if not check back the previous step for accurate environment creation.

Step 5: Download the [simulator here](https://github.com/udacity/FCND-Simulator-Releases/releases)

Step 6: Now, start the simulator and run the backyard_flyer python scrip

```
python backyard_flyer.py
```

Supported Sytems: MacOS, Windows, Linux

## Troubleshooting

**NOTE:** If `future` is not installed prior to `pymavlink` this will output an error message similar to the following:

```
  # omitted Traceback
  ModuleNotFoundError: No module named 'future'
  
  ----------------------------------------
  Failed building wheel for pymavlink
  Running setup.py clean for pymavlink
Failed to build pymavlink
Installing collected packages: torchfile, pillow, idna, chardet, urllib3, requests, visdom, future, lxml, pymavlink, utm, websockets, uvloop, udacidrone
  Running setup.py install for pymavlink ... done
  Running setup.py install for udacidrone ... done
Successfully installed chardet-3.0.4 future-0.16.0 idna-2.6 lxml-4.1.1 pillow-5.0.0 pymavlink-2.2.8 requests-2.18.4 torchfile-0.1.0 udacidrone-0.1.0 urllib3-1.22 utm-0.4.2 uvloop-0.9.1 visdom-0.1.7 websockets-4.0.1
```

If the output is `Successfully installed ... pymavlink ..` it's likely the install is fine. You can check by running:

```
python -c "import pymavlink"
```

if the install was successful there should be no output.

**NOTE:** If you're on MacOS you'll need to install developer tools to install `pymavlink`. You can do this by typing the following into a shell:

```
xcode-select --install
```

**NOTE:** On Windows you may need to open a terminal/powershell as an administrator. This can be done by right-clicking the program and selecting "Run as Administrator".

