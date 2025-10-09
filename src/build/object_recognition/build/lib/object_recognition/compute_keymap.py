import yaml
import math

layout = [
    (["`", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "-", "=", "BACKSPACE"],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,2]),

    (["TAB", "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P", "[", "]", "\\"],
        [1.25,1,1,1,1,1,1,1,1,1,1,1,1,1.25]),

    (["CAPS", "A", "S", "D", "F", "G", "H", "J", "K", "L", ";", "'", "ENTER"],
        [1.75,1,1,1,1,1,1,1,1,1,1,1,2.25]),
    
    (["LSHIFT", "Z", "X", "C", "V", "B", "N", "M", ",", ".", "/", "RSHIFT"],
        [2.25,1,1,1,1,1,1,1,1,1,1,1.25]),

    (["LCTRL", "LWIN", "LALT", "SPACE", "RALT", "FN", "MENU", "RCTRL"],
        [1.25,1.25,1.25,6.25,1.25,1.25,1.25,1.25])
]

key_pitch = 0.019 # meters
spacing = 0.002 # meters

def compute_row_layout(key_widths, spacing=0.0):
    total_row_width = sum(key_widths) + spacing * (len(key_widths) - 1)
    x_positions = []
    current_x = -total_row_width / 2.0 + key_widths[0] / 2.0
    x_positions.append(round(current_x, 6))
    for i in range(1, len(key_widths)):
        current_x += (key_widths[i-1] / 2.0) + spacing + (key_widths[i] / 2.0)
        x_positions.append(round(current_x, 6))
    return x_positions

keymap = {}
num_rows = len(layout)
for row_index, (keys, widths_u) in enumerate(layout):
    widths_m = [w * key_pitch for w in widths_u]
    x_positions = compute_row_layout(widths_m, spacing)
    y_position = ((num_rows -1) / 2.0 - row_index) * key_pitch
    for key, x, w in zip(keys, x_positions, widths_m):
        keymap[key] = {
            'x': x,
            'y': round(y_position, 6),
            'z_top': 0.0,
            'width': w,
            'height': 0.018
        }

with open('keymap.yaml', 'w') as f:
    yaml.dump(keymap, f, sort_keys=False)

print("Keymap saved to keymap.yaml")