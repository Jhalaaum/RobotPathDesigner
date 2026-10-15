# Robot Path Designer

A visual route-planning and code-generation tool for LEGO SPIKE Prime robots using Pybricks.

## Setup

### 1. Clone the repository

Clone this repository using GitHub Desktop.

### 2. Create a virtual environment

macOS / Linux:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

Windows (PowerShell):

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
```

### 3. Install dependencies

```bash
python -m pip install -r requirements.txt
```

### 4. Run the application

```bash
python main.py
```

## Collaboration

- Create a separate branch for each feature.
- Open pull requests before merging into `main`.
- Do not commit `.venv` or generated cache files.
- Update `requirements.txt` when adding dependencies.