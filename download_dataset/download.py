import os
from dotenv import load_dotenv
from huggingface_hub import login, snapshot_download



def main():

    # .env 파일 로드
    load_dotenv()

    hf_api_token = os.getenv("HF_TOKEN")

    login(token=hf_api_token)

    # # Download an entire repository
    # snapshot_download(
    #   repo_id="nvidia/PhysicalAI-Autonomous-Vehicles-NuRec", 
    #   repo_type="dataset"
    # )

    # # Download all the files in a folder
    # snapshot_download(
    #   repo_id="nvidia/PhysicalAI-Autonomous-Vehicles-NuRec", 
    #   repo_type="dataset", 
    #   allow_patterns="sample_set/25.07_release/Batch0002/001b28cb-b8f7-4627-ae65-fda88612d5bf/*",
    #   local_dir="./autovehicle_dataset"
    # )

    snapshot_download(
      repo_id="nvidia/PhysicalAI-Robotics-NuRec", 
      repo_type="dataset", 
      allow_patterns="nova_carter-galileo/*", # 특정 폴더만 지정
      local_dir="./robotics_dataset"
    )

    # Download an individual file
    # snapshot_download(
    #     repo_id="nvidia/PhysicalAI-Autonomous-Vehicles-NuRec", 
    #     repo_type="dataset", 
    #     allow_patterns="sample_set/25.07_release/Batch0002/001b28cb-b8f7-4627-ae65-fda88612d5bf/001b28cb-b8f7-4627-ae65-fda88612d5bf.usdz",
    #     local_dir="./autovehicle_dataset"
    # )

if __name__ == "__main__":
    main()