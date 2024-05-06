import os
import requests
import cloudinary
import cloudinary.api
import cloudinary.uploader
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

cloudinary.config(
    cloud_name=os.getenv("CLOUD_NAME"),
    api_key=os.getenv("API_KEY"),
    api_secret=os.getenv("API_SECRET")
)

def get_cloudinary_image_urls(project_name):
    try:
        image_data = cloudinary.api.resources(type='upload', prefix="files"+"/"+project_name, max_results=500)
        image_data = image_data["resources"]
        urls = [item['url'] for item in image_data]
        return urls
    except:
        return []

def download_image(url, folder_path):
    try:
        # Send an HTTP request to the URL
        response = requests.get(url)

        # Ensure the request was successful (status code 200)
        if response.status_code == 200:
            # Extract the filename from the URL
            filename = os.path.join(folder_path, os.path.basename(url))
            
            # Save the image to the local folder
            with open(filename, 'wb') as f:
                f.write(response.content)
            print(f"Downloaded: {filename}")
        else:
            print(f"Failed to download: {url}, status code: {response.status_code}")
    except Exception as e:
        print(f"Error downloading {url}: {e}")

def get_image_from_cloudinary(images_folder,project_name):
    
    image_urls = get_cloudinary_image_urls(project_name)
    
    if(len(image_urls)==0):
        print("No image found")

    if not os.path.exists(images_folder):
        os.makedirs(images_folder)

    for url in image_urls:
        download_image(url, images_folder)

def upload_to_cloudinary(file_path):
    try:
        # Upload the file to Cloudinary
        upload_result = cloudinary.uploader.upload(file_path)

        # Extract the public ID and URL of the uploaded file
        public_id = upload_result['public_id']
        url = upload_result['secure_url']

        return {'status': 'success', 'public_id': public_id, 'url': url}
    except Exception as e:
        return {'status': 'error', 'message': str(e)}