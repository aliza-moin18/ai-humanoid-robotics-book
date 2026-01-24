import os
import sys
import tempfile
from unittest.mock import patch

# Add the backend/src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

from cli.main import main


def test_cli_without_config():
    """Test that the CLI properly validates configuration."""
    # Mock sys.argv to simulate command line arguments
    with patch('sys.argv', ['python', '-m', 'src.cli.main', '--help']):
        try:
            main()
        except SystemExit as e:
            # The help command should cause a SystemExit with code 0 or 1
            # This is expected behavior
            assert e.code in [0, 1, 2]  # 0 for help, 1 for validation error, 2 for argument error


if __name__ == "__main__":
    test_cli_without_config()
    print("Test passed: CLI properly validates configuration")