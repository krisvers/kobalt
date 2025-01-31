# [***high priority***]

- ### Pipeline and Pipeline State:
    - createBlendAttachmentState
    - createBlendState
    - createPipelineResourcePool
    - allocatePipelineResourceSet
    - updatePipelineResourceSet

- ### Commands:
    - bindIndexBuffer
    - drawIndexed

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
    - downloadBufferData
    - downloadTextureData
    - copyTexture
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
