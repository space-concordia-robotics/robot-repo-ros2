# scripts/build_dataset.py
import os, yaml
from autodistill_grounded_sam import GroundedSAM
from autodistill.detection import CaptionOntology

#put raw jpg/png images here:
IN_DIR = os.path.abspath("datasets/images")
OUT_DIR = os.path.abspath("datasets/ground_objects")

ontology = CaptionOntology({
    "an orange hammer": "hammer",
    "a plastic water bottle": "water_bottle",
})

os.makedirs(IN_DIR, exist_ok=True)
os.makedirs(OUT_DIR, exist_ok=True)

base_model = GroundedSAM(
    ontology=ontology,
    box_threshold=0.35,
    text_threshold=0.25,
    nms_threshold=0.5,
    device="cuda"  # or "cpu"
)

base_model.label(
    input_folder=IN_DIR,
    output_folder=OUT_DIR,
    extension=".png",
    split_fractions=(0.8, 0.2, 0.0)
)

names = list(ontology.classes())
with open(os.path.join(OUT_DIR, "data.yaml"), "w") as f:
    yaml.safe_dump({
        "path": OUT_DIR,
        "train": "train/images",
        "val": "valid/images",
        "nc": len(names),
        "names": names
    }, f, sort_keys=False)
# output: datasets/ground_objects/data.yaml
print("Dataset ready:", OUT_DIR)
print("Classes:", names)
