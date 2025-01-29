# [***high priority***]

- ### Pipeline and Pipeline State:
    - createBlendAttachmentState
    - createBlendState
    - createPipelineResourceLayout
    - createPipelineResourcePool
    - allocatePipelineResourceSet
    - updatePipelineResourceSet

- ### Resources:
    - uploadBufferData
    - uploadTextureData

- ### Commands:
    - bindIndexBuffer
    - drawIndexed
    - pushDynamicPipelineResources

# (*moderate priority*)
- ### Render Passes:
    - createRenderAttachmentState
    - createRenderSubpass
    - createRenderPass

- ### Pipeline and Pipeline State:
    - createTessellationState
    - allocatePipelineResourceSets
    - storePipeline
    - loadPipeline
    - createComputePipeline

- ### Resources:
    - copyBuffer
    - copyTexture
    - copyTextureFromBuffer
    - copyBufferFromTexture
    - createBufferView

- ### Commands:
    - pushConstants
    - copyBuffer
    - nextSubpass
    - beginRenderPass 

# low priority

- ### Synchronization
    - waitForDevice
    - waitForQueue

- ### Command List
    - createCommandList (secondary)
    - recordSecondaryCommands
