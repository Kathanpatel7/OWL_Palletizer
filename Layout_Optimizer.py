import matplotlib.pyplot as plt
import matplotlib.patches as patches

def can_place_box(x, y, box_width, box_height, pallet_size, placed_items):
    """Check if a box can be placed at the given position."""
    if x + box_width > pallet_size[0] or y + box_height > pallet_size[1]:
        return False
    return not any(
        (x < px + pw and x + box_width > px and y < py + ph and y + box_height > py)
        for px, py, pw, ph in placed_items
    )

def find_next_position(pallet_size, box_dim, placed_items):
    """Find the next position where a box can be placed."""
    for y in range(pallet_size[1] - box_dim[1] + 1):
        for x in range(pallet_size[0] - box_dim[0] + 1):
            if can_place_box(x, y, box_dim[0], box_dim[1], pallet_size, placed_items):
                return (x, y)
    return None

def optimize_box_placement(num_boxes, box_dim, pallet_size):
    placed_items = []
    for _ in range(num_boxes):
        position = find_next_position(pallet_size, box_dim, placed_items)
        if position:
            placed_items.append((position[0], position[1], box_dim[0], box_dim[1]))
        else:
            # Try with rotated box
            rotated_box_dim = (box_dim[1], box_dim[0])
            position = find_next_position(pallet_size, rotated_box_dim, placed_items)
            if position:
                placed_items.append((position[0], position[1], rotated_box_dim[0], rotated_box_dim[1]))
            else:
                break  # No more space on the pallet

    return placed_items

def get_center_coordinates(placed_items):
    """Get center coordinates for each placed box."""
    center_coordinates = []
    for item in placed_items:
        center_x = item[0] + item[2] / 2
        center_y = item[1] + item[3] / 2
        center_coordinates.append((center_x, center_y))
    return center_coordinates

def plot_pallet(pallet_size, items):
    fig, ax = plt.subplots()
    pallet = patches.Rectangle((0, 0), pallet_size[0], pallet_size[1], edgecolor='black', facecolor='none')
    ax.add_patch(pallet)

    for item in items:
        rect = patches.Rectangle((item[0], item[1]), item[2], item[3], edgecolor='blue', facecolor='lightblue')
        ax.add_patch(rect)

    center_coordinates = get_center_coordinates(items)
    for center in center_coordinates:
        plt.scatter(center[0], center[1], color='red', marker='x')

    plt.xlim(0, pallet_size[0])
    plt.ylim(0, pallet_size[1])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

# User input for number of boxes and dimensions
num_boxes = int(input("Enter the number of boxes: "))
box_width = int(input("Enter the box width: "))
box_height = int(input("Enter the box height: "))

# Define pallet size (width, height)
pallet_size = (120, 120)

# Optimize box placement
placed_items = optimize_box_placement(num_boxes, (box_width, box_height), pallet_size)

# Get the center coordinates
center_coordinates = get_center_coordinates(placed_items)

# Output the center coordinates
print("Center coordinates of each box:")
for center in center_coordinates:
    print(f"({center[0]}, {center[1]})")

# Plot the pallet with items and center coordinates
plot_pallet(pallet_size, placed_items)

# Output the number of boxes that could be placed
print(f"Number of boxes that could be placed: {len(placed_items)}")

