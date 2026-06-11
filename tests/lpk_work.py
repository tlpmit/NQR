import warnings

from tests.test_perception_obm import test_obm_on_kinsim_images

warnings.filterwarnings("ignore", category=SyntaxWarning, module="colormath.*")

test_obm_on_kinsim_images(mode="display")
