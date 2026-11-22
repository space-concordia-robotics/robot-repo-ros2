# Science Payload

This package provides two nodes for interfacing with the science payload.

## Spectrometer Service Node

The spectrometer service node, `science_payload`, starts an action server listening to `/rover/science/scan` for scan requests.
It takes no parameters.

## Science Collection Node

The science collection node, `science_collection`, executes a scan using `/rover/science/scan`.

It takes the following parameters:

| Name              | Description                                                                                 |
|-------------------|---------------------------------------------------------------------------------------------|
| `site`            | The site index.                                                                             |
| `cuvette`         | The cuvette index.                                                                          |
| `sample`          | The sample type, one of `baseline`, `ph`, or `resazurin`.                                   |
| `characteristic`  | The characteristic being tested.                                                            |
| `light_pollution` | Whether to load the baseline sample with light pollution or the one with no light pollution |
