# test/conftest.py
import pytest
from utils.logger import setup_logging


@pytest.fixture(scope="session", autouse=True)
def initialize_logging_for_tests():
    setup_logging()
