from perception import (
    extract_detection_metadata,
    extract_quanser_detection_metadata,
    extract_sign_detection_metadata,
    is_orange,
)


def process_camera_models(qcar_img, cone_yolo, seg_yolo):
    qcar_img.read()
    rgb_processed_cones = cone_yolo.pre_process(qcar_img.rgb)
    prediction_cones = cone_yolo.predict(
        inputImg=rgb_processed_cones,
        classes=[0],
        confidence=0.05,
        half=True,
        verbose=False,
    )
    res_cones = (
        prediction_cones[0]
        if isinstance(prediction_cones, list)
        else prediction_cones
    )
    raw_results_cones = cone_yolo.post_processing(
        alignedDepth=qcar_img.depth, clippingDistance=5
    )

    keep_indices, filtered_res_cones, results_cones, filtered_cone_metadata = (
        [],
        None,
        [],
        [],
    )

    if (
        res_cones is not None
        and res_cones.boxes is not None
        and len(res_cones.boxes) > 0
    ):
        cone_metadata = extract_detection_metadata(res_cones)
        for i, box in enumerate(res_cones.boxes.xyxy.cpu().numpy()):
            if is_orange(rgb_processed_cones, box):
                keep_indices.append(i)
                if i < len(cone_metadata):
                    filtered_cone_metadata.append(cone_metadata[i])

        if keep_indices:
            filtered_res_cones = res_cones[keep_indices]
            for i in keep_indices:
                if i < len(raw_results_cones):
                    results_cones.append(raw_results_cones[i])

    rgb_processed_seg = seg_yolo.pre_process(qcar_img.rgb)
    prediction_seg = seg_yolo.predict(
        inputImg=rgb_processed_seg, confidence=0.30, half=True, verbose=False
    )
    res_seg = (
        prediction_seg[0]
        if isinstance(prediction_seg, list)
        else prediction_seg
    )
    results_seg = seg_yolo.post_processing(
        alignedDepth=qcar_img.depth, clippingDistance=5
    )

    sign_metadata = extract_sign_detection_metadata(res_seg)
    traffic_light_metadata = extract_quanser_detection_metadata(results_seg)

    return (
        filtered_cone_metadata + sign_metadata + traffic_light_metadata,
        filtered_res_cones,
    )


def render_camera_results(seg_yolo, filtered_res_cones):
    combined_img = seg_yolo.post_process_render(showFPS=True)
    if filtered_res_cones is not None and len(filtered_res_cones) > 0:
        combined_img = filtered_res_cones.plot(img=combined_img)
    return combined_img
