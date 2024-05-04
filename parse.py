import backend

def parser():
    backend.download_image("https://res.cloudinary.com/drdueyqsi/raw/upload/v1712749540/ProjectNames/sample.txt","./")

    with open('sample.txt', 'r') as file:
        content = file.read()

    # Split the content by 'and'
    parts = content.split('and')

    # Remove extra whitespace and get the variables
    project_name = parts[0].strip()
    number_of_images_in_one_rotation = parts[1].strip()

    return {project_name,number_of_images_in_one_rotation}
    