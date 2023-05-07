import os
import fnmatch
import shutil
import xml.etree.ElementTree as ET

texture_directory = "./materials/textures"
template_directory = "./template/model"


def get_textures_from(directory):
    textures = []
    for filename in os.listdir(directory):
        if fnmatch.fnmatch(filename, "*.png"):
            textures.append(filename[:-4])
    return textures


def initialize_directory(texture_name):
    if os.path.exists(texture_name):
        shutil.rmtree(texture_name)
    os.mkdir(texture_name)


def replace_in_xml(path, tag, value):
    tree = ET.parse(path)
    root = tree.getroot()
    element = root.find(tag)
    element.text = value
    tree.write(path)


def replace_line(path, line_index, value):
    with open(path, "r") as f:
        lines = f.readlines()
    lines[line_index - 1] = value
    with open(path, "w") as f:
        f.writelines(lines)


def replace_init_from(path, value):
    replace_line(path, 14, f"      <init_from>{value}</init_from>\n")


def replace_model_name(path, value):
    tree = ET.parse(path)
    root = tree.getroot()
    model_elem = root.find("./model")
    model_elem.set("name", value)
    tree.write(path)


def generate_model_config_file(texture_name):
    source_file = os.path.join(template_directory, "model.config")
    dest_file = os.path.join(texture_name, "model.config")
    shutil.copyfile(source_file, dest_file)
    replace_in_xml(dest_file, "name", texture_name)


def generate_model_sdf_file(texture_name):
    source_file = os.path.join(template_directory, "model.sdf")
    dest_file = os.path.join(texture_name, "model.sdf")
    shutil.copyfile(source_file, dest_file)
    replace_model_name(dest_file, texture_name)
    replace_in_xml(dest_file, ".//uri", f"{texture_name}.dae")


def generate_mesh_file(texture_name):
    source_file = os.path.join(template_directory, "Marker0.dae")
    dest_file = os.path.join(texture_name, f"{texture_name}.dae")
    shutil.copyfile(source_file, dest_file)
    replace_init_from(dest_file, f"{texture_name}.png")


if __name__ == "__main__":
    textures = get_textures_from(texture_directory)
    print(f"Generating {len(textures)} models...")
    for texture_name in textures:
        initialize_directory(texture_name)
        generate_model_config_file(texture_name)
        generate_model_sdf_file(texture_name)
        generate_mesh_file(texture_name)
    print("Done")
