# Jupyter Notebook

Jupyter is a free, open-source, interactive web tool known as a computational notebook, which researchers can use to combine software code, computational output, explanatory text and multimedia resources in a single document.

## Launch a Jupyter Notebook App (Linux, MacOS)

 - For Linux and MacOS systems, open a new terminal window.
 - Enter the startup folder by typing `cd /some folder name`
 - Type `jupyter notebook` to launch the Jupyter Notebook App. The notebook interface will appear in a new browser window or tab.

## Launch a Jupyter Notebook App (Windows)

Double-click on the Jupyter Notebook desktop launcher (icon shows [IPy]) to start the Jupyter Notebook App. The notebook interface will appear in a new browser window or tab. A secondary terminal window (used only for error logging and for shut down) will be also opened.

## Executing a notebook

 - Launch the Jupyter Notebook App (see previous section).
 - In the Notebook Dashboard navigate to find the notebook: clicking on its name will open it in a new browser tab.
 - Click on the menu Help -> User Interface Tour for an overview of the Jupyter Notebook App user interface.
 - You can run the notebook document step-by-step (one cell a time) by pressing shift + enter.
 - You can run the whole notebook in a single step by clicking on the menu Cell -> Run All.
 - To restart the kernel (i.e. the computational engine), click on the menu Kernel -> Restart. This can be useful to start over a computation from scratch (e.g. variables are deleted, open files are closed, etc…).

## Closing a notebook

When a notebook is opened, its “computational engine” (called the kernel) is automatically started. Closing the notebook browser tab, will not shut down the kernel, instead the kernel will keep running until is explicitly shut down.

To shut down a kernel, go to the associated notebook and click on menu File -> Close and Halt. Alternatively, the Notebook Dashboard has a tab named Running that shows all the running notebooks (i.e. kernels) and allows shutting them down (by clicking on a Shutdown button).

## Shut down the Jupyter Notebook App

Closing the browser (or the tab) will not close the Jupyter Notebook App. To completely shut it down you need to close the associated terminal.