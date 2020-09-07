# Usage in Jupyter notebook

1. Clone this directory
2. Create an environment from jupyter_gui.yaml
3. Launch a Notebook within this directory
4. Execute the following code in a cell:

```
	from eegstream import EEG
	from gui import GUI
	%matplotlib notebook
	gui = GUI()
	gui.display()
```
