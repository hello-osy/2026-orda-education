"""4번 전용: osy-260809의 BEV/차선 추적 계산을 ROS 없이 실행한다.

Source commit: 3a29efdae1467355c1d7f059f92238febf85a33a
원문은 reference_osy_260809에 보관한다. ROS 입출력·로그만 GUI 반환값으로
바꾸고, BEV/마스크/후보 선택/폴백/조향 계산과 기본값은 보존했다.
"""
import cv2
import numpy as np


IMAGE_TOPIC = '/camera/high/image_raw'
LANE_OFFSET_TOPIC = '/lane_offset'
DEBUG_IMAGE_TOPIC = '/lane_offset/debug_image_ngg'
BEV_Y_TOP_RATIO = 0.4
BEV_Y_BOTTOM_RATIO = 1.0
BEV_TOP_WIDTH_RATIO = 1.0
BEV_BOTTOM_WIDTH_RATIO = 0.7
BEV_OUTPUT_WIDTH = 640
BEV_OUTPUT_HEIGHT = 0
HORIZONTAL_RUN_MIN_PX = 50
HORIZONTAL_ERASE_HALF_BAND_PX = 5
GREEN_NEAR_DISTANCE_PX = 20
GREEN_MIN_PIXELS = 10
GREEN_RIGHT_MARGIN_PX = -1000
MIN_COMPONENT_AREA = 50
MIN_LINE_HEIGHT_PX = 25
MIN_LINE_ASPECT_RATIO = 0.0
SMALL_COMPACT_MAX_AREA = 1800
SMALL_COMPACT_MAX_SIDE_PX = 90
SMALL_COMPACT_MIN_ELONGATION = 2.0
TARGET_RIGHT_X = 505
RIGHT_GREEN_STABLE_FRAMES = 10
TARGET_CENTER_X = 108
CENTER_SEARCH_HALF_WIDTH_PX = 150
CENTER_MAX_X_JUMP_PX = 35
CENTER_MAX_OFFSET_STEP = 8
CENTER_NUM_WINDOWS = 15
CENTER_WINDOW_MARGIN_PX = 160
CENTER_WINDOW_MIN_COMPONENT_PIXELS = 30
CENTER_RECONNECT_HEIGHT_PX = 15
CENTER_MAX_TRACKED_PIECES = 4
CENTER_MAX_VERTICAL_OVERLAP_RATIO = 0.20
NEAR_ROWS = 80
NEAR_MIN_PIXELS = 20
ALLOW_LINE_BOTTOM_FALLBACK = True
OFFSET_ERROR_LIMIT_PX = 130
LANE_OFFSET_LIMIT = 45
MAX_OFFSET_JUMP = 2000
OFFSET_SMOOTHING_ALPHA = 1.0
DEBUG_VIEW = False
WINDOW_NAME = 'timed_lane_offset_ngg'
WHITE_MASK_WINDOW_NAME = 'ngg_white_mask'
GREEN_MASK_WINDOW_NAME = 'ngg_green_mask'


DEFAULTS = {
    'bev_y_top_ratio': BEV_Y_TOP_RATIO,
    'bev_y_bottom_ratio': BEV_Y_BOTTOM_RATIO,
    'bev_top_width_ratio': BEV_TOP_WIDTH_RATIO,
    'bev_bottom_width_ratio': BEV_BOTTOM_WIDTH_RATIO,
    'bev_output_width': BEV_OUTPUT_WIDTH,
    'bev_output_height': BEV_OUTPUT_HEIGHT,
    'horizontal_run_min_px': HORIZONTAL_RUN_MIN_PX,
    'horizontal_erase_half_band_px': HORIZONTAL_ERASE_HALF_BAND_PX,
    'green_near_distance_px': GREEN_NEAR_DISTANCE_PX,
    'green_min_pixels': GREEN_MIN_PIXELS,
    'green_right_margin_px': GREEN_RIGHT_MARGIN_PX,
    'min_component_area': MIN_COMPONENT_AREA,
    'min_line_height_px': MIN_LINE_HEIGHT_PX,
    'min_line_aspect_ratio': MIN_LINE_ASPECT_RATIO,
    'small_compact_max_area': SMALL_COMPACT_MAX_AREA,
    'small_compact_max_side_px': SMALL_COMPACT_MAX_SIDE_PX,
    'small_compact_min_elongation': SMALL_COMPACT_MIN_ELONGATION,
    'target_right_x': TARGET_RIGHT_X,
    'right_green_stable_frames': RIGHT_GREEN_STABLE_FRAMES,
    'target_center_x': TARGET_CENTER_X,
    'center_search_half_width_px': CENTER_SEARCH_HALF_WIDTH_PX,
    'center_max_x_jump_px': CENTER_MAX_X_JUMP_PX,
    'center_max_offset_step': CENTER_MAX_OFFSET_STEP,
    'center_num_windows': CENTER_NUM_WINDOWS,
    'center_window_margin_px': CENTER_WINDOW_MARGIN_PX,
    'center_window_min_component_pixels': CENTER_WINDOW_MIN_COMPONENT_PIXELS,
    'center_reconnect_height_px': CENTER_RECONNECT_HEIGHT_PX,
    'center_max_tracked_pieces': CENTER_MAX_TRACKED_PIECES,
    'center_max_vertical_overlap_ratio': CENTER_MAX_VERTICAL_OVERLAP_RATIO,
    'near_rows': NEAR_ROWS,
    'near_min_pixels': NEAR_MIN_PIXELS,
    'allow_line_bottom_fallback': ALLOW_LINE_BOTTOM_FALLBACK,
    'offset_error_limit_px': OFFSET_ERROR_LIMIT_PX,
    'lane_offset_limit': LANE_OFFSET_LIMIT,
    'max_offset_jump': MAX_OFFSET_JUMP,
    'offset_smoothing_alpha': OFFSET_SMOOTHING_ALPHA,
    'debug_view': DEBUG_VIEW,
}


class CompetitionLaneTracker:
    def __init__(self):
        self._load_parameters(DEFAULTS.__getitem__)
        self.reset()

    def reset(self):
        self.last_offset = 0.0
        self.last_line_x = None
        self.last_center_line_x = None
        self.active_control_line_kind = None
        self.right_green_stable_count = 0
        self.output = 0
        self.result = None
        self.publish_debug_image = True


    def _load_parameters(self, get):
        self.bev_y_top_ratio = float(np.clip(get('bev_y_top_ratio'), 0.0, 1.0))
        self.bev_y_bottom_ratio = float(
            np.clip(get('bev_y_bottom_ratio'), self.bev_y_top_ratio, 1.0)
        )
        self.bev_top_width_ratio = float(
            np.clip(get('bev_top_width_ratio'), 0.0, 1.0)
        )
        self.bev_bottom_width_ratio = float(
            np.clip(get('bev_bottom_width_ratio'), 0.0, 1.0)
        )
        self.bev_output_width = max(1, int(get('bev_output_width')))
        self.bev_output_height = max(0, int(get('bev_output_height')))
        self.horizontal_run_min_px = max(1, int(get('horizontal_run_min_px')))
        self.horizontal_erase_half_band_px = max(
            0, int(get('horizontal_erase_half_band_px'))
        )
        self.green_near_distance_px = max(1, int(get('green_near_distance_px')))
        self.green_min_pixels = int(get('green_min_pixels'))
        self.green_right_margin_px = float(get('green_right_margin_px'))
        self.min_component_area = int(get('min_component_area'))
        self.min_line_height_px = int(get('min_line_height_px'))
        self.min_line_aspect_ratio = float(get('min_line_aspect_ratio'))
        self.small_compact_max_area = max(
            0, int(get('small_compact_max_area'))
        )
        self.small_compact_max_side_px = max(
            0, int(get('small_compact_max_side_px'))
        )
        self.small_compact_min_elongation = max(
            1.0, float(get('small_compact_min_elongation'))
        )
        self.target_right_x = int(get('target_right_x'))
        self.right_green_stable_frames = max(
            1, int(get('right_green_stable_frames'))
        )
        self.target_center_x = int(get('target_center_x'))
        self.center_search_half_width_px = max(
            1, int(get('center_search_half_width_px'))
        )
        self.center_max_x_jump_px = max(
            1, int(get('center_max_x_jump_px'))
        )
        self.center_max_offset_step = max(
            1, int(get('center_max_offset_step'))
        )
        self.center_num_windows = max(1, int(get('center_num_windows')))
        self.center_window_margin_px = max(
            1, int(get('center_window_margin_px'))
        )
        self.center_window_min_component_pixels = max(
            1, int(get('center_window_min_component_pixels'))
        )
        self.center_reconnect_height_px = max(
            1, int(get('center_reconnect_height_px'))
        )
        self.center_max_tracked_pieces = max(
            1, int(get('center_max_tracked_pieces'))
        )
        self.center_max_vertical_overlap_ratio = float(np.clip(
            get('center_max_vertical_overlap_ratio'), 0.0, 1.0
        ))
        self.near_rows = max(1, int(get('near_rows')))
        self.near_min_pixels = int(get('near_min_pixels'))
        self.allow_line_bottom_fallback = bool(get('allow_line_bottom_fallback'))
        self.offset_error_limit_px = max(1, int(get('offset_error_limit_px')))
        self.lane_offset_limit = max(1, int(get('lane_offset_limit')))
        self.max_offset_jump = int(get('max_offset_jump'))
        self.offset_smoothing_alpha = float(
            np.clip(get('offset_smoothing_alpha'), 0.0, 1.0)
        )
        self.debug_view = bool(get('debug_view'))

    def process(self, segmented):
        bev = self.warp_to_bev(segmented)
        white_mask = self.make_white_mask(bev)
        green_mask = self.make_green_mask(bev)
        self.white_mask = white_mask
        self.green_mask = green_mask

        right_mask, right_x, right_mode, right_reason = self.find_right_solid_line(
            white_mask, green_mask
        )
        if right_x is None:
            self.right_green_stable_count = 0
        else:
            self.right_green_stable_count = min(
                self.right_green_stable_count + 1,
                self.right_green_stable_frames,
            )

        target_x = self.target_right_x
        line_kind = 'RIGHT'
        line_mask = right_mask
        measured_x = right_x
        measure_mode = right_mode
        reject_reason = right_reason
        center_jump_rejected = False

        if self.right_green_stable_count < self.right_green_stable_frames:
            # 오른쪽 실선은 초록 매트가 연속 프레임에서 확인될 때만 제어권을
            # 받는다. 그 전에는 중앙 점선을 계속 사용해 짧은 초록 오검출에
            # 조향 기준이 바뀌지 않게 한다.
            if self.active_control_line_kind != 'CENTER':
                # 오른쪽 실선 주행 중 남아 있던 중앙선 좌표는 다음 중앙선 주행의
                # 기준으로 쓰면 안 된다. 전환 프레임은 새 중앙선 트랙의 시작이다.
                self.last_center_line_x = None
            target_x = self.target_center_x
            line_kind = 'CENTER'
            measured_x = None
            measure_mode = 'none'
            center_mask, center_x, center_mode, center_reason = self.find_center_line(
                white_mask
            )
            # 새 중앙선 후보가 점프 판정으로 버려져도, 디버그 화면에서는 왜
            # 버렸는지 확인할 수 있도록 후보 마스크는 표시한다.
            if center_mask is not None:
                line_mask = center_mask
            if (
                center_x is not None
                and self.last_center_line_x is not None
                and abs(center_x - self.last_center_line_x)
                > self.center_max_x_jump_px
            ):
                center_jump_rejected = True
                center_reason = (
                    f'center x jump {self.last_center_line_x:.1f} -> '
                    f'{center_x:.1f} exceeds {self.center_max_x_jump_px}px'
                )
                center_x = None
            if center_x is not None:
                line_mask = center_mask
                measured_x = center_x
                measure_mode = center_mode
                target_x = self.target_center_x
                line_kind = 'CENTER'
                if right_x is not None:
                    reject_reason = (
                        f'right green stabilizing '
                        f'({self.right_green_stable_count}/'
                        f'{self.right_green_stable_frames})'
                    )
            else:
                reject_reason = f'{reject_reason}; center fallback: {center_reason}'

        if measured_x is None:
            # 중앙선도 잠깐 끊기거나, 오른쪽 실선의 10프레임 안정화가 아직 끝나지
            # 않았으면 새 offset을 만들지 않고 직전 값을 그대로 유지한다.
            self.publish_offset(self.last_offset)
            self.publish_debug(
                bev,
                'CENTER JUMP HOLD' if center_jump_rejected else 'CENTER LOST HOLD',
                line_mask, None, measure_mode,
                target_x=target_x, line_kind=line_kind,
                held_x=(
                    self.last_center_line_x if line_kind == 'CENTER' else None
                ),
            )
            return self.result

        raw_offset = self.map_line_x_to_offset(measured_x, target_x)
        center_step_limited = False
        if line_kind == 'CENTER':
            limited_offset = float(np.clip(
                raw_offset,
                self.last_offset - self.center_max_offset_step,
                self.last_offset + self.center_max_offset_step,
            ))
            if limited_offset != raw_offset:
                center_step_limited = True
                raw_offset = int(round(limited_offset))

        if abs(raw_offset - self.last_offset) > self.max_offset_jump:
            self.publish_offset(self.last_offset)
            self.publish_debug(
                bev, 'JUMP REJECTED', line_mask, measured_x, measure_mode,
                raw_offset, target_x, line_kind,
            )
            return self.result

        self.last_offset = (
            self.offset_smoothing_alpha * raw_offset
            + (1.0 - self.offset_smoothing_alpha) * self.last_offset
        )
        if line_kind == 'RIGHT':
            self.last_line_x = measured_x
            # 다음에 중앙선 fallback으로 들어가면, 과거 중앙선이 아닌 새 관측부터
            # 중앙선 점프 제한을 시작한다.
            self.last_center_line_x = None
        else:
            self.last_center_line_x = measured_x
        self.active_control_line_kind = line_kind
        self.publish_offset(self.last_offset)
        self.publish_debug(
            bev,
            (
                'OK' if line_kind == 'RIGHT'
                else 'CENTER STEP LIMITED' if center_step_limited
                else 'CENTER FALLBACK'
            ),
            line_mask, measured_x, measure_mode, raw_offset, target_x, line_kind,
        )
        return self.result

    def make_white_mask(self, segmented_bev):
        white_mask = (
            np.all(segmented_bev == (255, 255, 255), axis=2).astype(np.uint8) * 255
        )
        return self.remove_horizontal_white_bands(white_mask)

    def remove_horizontal_white_bands(self, white_mask):
        """50px 이상 연속된 가로 흰 선과 그 위아래 밴드를 마스크에서 지운다."""
        height, width = white_mask.shape
        min_run = self.horizontal_run_min_px
        if width < min_run or height == 0:
            return white_mask

        # 각 행의 길이 min_run짜리 구간 합을 구한다. 합이 min_run이면 그 구간은
        # 전부 흰색이므로 해당 행에 연속된 가로선이 있다고 판단할 수 있다.
        binary = (white_mask > 0).astype(np.int32)
        row_prefix = np.pad(
            np.cumsum(binary, axis=1), ((0, 0), (1, 0)), mode='constant'
        )
        window_sums = row_prefix[:, min_run:] - row_prefix[:, :-min_run]
        horizontal_rows = np.any(window_sums >= min_run, axis=1)
        if not np.any(horizontal_rows):
            return white_mask

        # 검출 행마다 위/아래 half_band를 더한 행 전체를 0으로 만든다.
        half_band = self.horizontal_erase_half_band_px
        erase_rows = cv2.dilate(
            horizontal_rows.astype(np.uint8).reshape(-1, 1),
            np.ones((half_band * 2 + 1, 1), dtype=np.uint8),
        ).reshape(-1) > 0

        filtered = white_mask.copy()
        filtered[erase_rows, :] = 0
        return filtered

    @staticmethod
    def make_green_mask(segmented_bev):
        return (
            np.all(segmented_bev == (0, 255, 0), axis=2).astype(np.uint8) * 255
        )

    def warp_to_bev(self, segmented):
        output_height = self.compute_bev_output_height(segmented.shape)
        src_points, dst_points = self.compute_bev_points(
            segmented.shape, output_height
        )
        transform = cv2.getPerspectiveTransform(src_points, dst_points)
        # 분류 색이 섞여 새 색으로 생기지 않도록 최근접 보간을 사용한다.
        return cv2.warpPerspective(
            segmented,
            transform,
            (self.bev_output_width, output_height),
            flags=cv2.INTER_NEAREST,
        )

    def compute_bev_output_height(self, frame_shape):
        if self.bev_output_height > 0:
            return self.bev_output_height

        height, width = frame_shape[:2]
        source_height = height * (
            self.bev_y_bottom_ratio - self.bev_y_top_ratio
        )
        return max(1, round(self.bev_output_width * source_height / width))

    def compute_bev_points(self, frame_shape, output_height):
        height, width = frame_shape[:2]
        y_top = height * self.bev_y_top_ratio
        y_bottom = height * self.bev_y_bottom_ratio
        src_points = np.float32([
            [0, y_top],
            [width, y_top],
            [width, y_bottom],
            [0, y_bottom],
        ])

        center_x = self.bev_output_width / 2.0
        top_half_width = self.bev_output_width * self.bev_top_width_ratio / 2.0
        bottom_half_width = (
            self.bev_output_width * self.bev_bottom_width_ratio / 2.0
        )
        dst_points = np.float32([
            [center_x - top_half_width, 0],
            [center_x + top_half_width, 0],
            [center_x + bottom_half_width, output_height],
            [center_x - bottom_half_width, output_height],
        ])
        return src_points, dst_points

    def find_right_solid_line(self, white_mask, green_mask):
        """초록 매트가 오른쪽에 붙은 흰 실선을 찾아 (마스크, 측정x, 모드, 사유) 반환.

        - 흰색 덩어리 중 크기/모양 필터를 통과한 것만 후보로 본다.
        - 후보를 green_near_distance_px 만큼 팽창시킨 이웃에서 초록 픽셀을 세고,
          그 초록의 평균 x가 덩어리 평균 x보다 오른쪽이어야 오른쪽 실선으로 인정한다.
          (중앙 점선은 양옆이 아스팔트라 여기서 걸러진다.)
        - 여러 개면 직전 측정 x에 가장 가까운 것을, 없으면 가장 큰 것을 고른다.
        """
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            white_mask, connectivity=8
        )
        if num_labels <= 1:
            return None, None, 'none', 'no white component'

        kernel_size = self.green_near_distance_px * 2 + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        green_bool = green_mask > 0

        best = None  # (score, label)
        shape_pass = 0
        for label in range(1, num_labels):
            x, y, w, h, area = stats[label]
            if area < self.min_component_area:
                continue
            if h < self.min_line_height_px:
                continue
            if w > 0 and (h / float(w)) < self.min_line_aspect_ratio:
                continue

            # 실제 회전 차선은 축 정렬 bbox만 보면 넓어질 수 있으므로
            # minAreaRect의 긴 변/짧은 변으로 길쭉함을 판단한다. 단, 이
            # 필터는 작은 객체에만 적용해 큰 대각선 차선을 보호한다.
            comp = labels == label
            if (
                area <= self.small_compact_max_area
                and max(w, h) <= self.small_compact_max_side_px
                and self.component_elongation(comp)
                < self.small_compact_min_elongation
            ):
                continue
            shape_pass += 1

            neighborhood = cv2.dilate(comp.astype(np.uint8), kernel) > 0
            green_near = green_bool & neighborhood
            green_count = int(np.count_nonzero(green_near))
            if green_count < self.green_min_pixels:
                continue
            # 초록이 덩어리보다 오른쪽에 있어야 한다.
            green_mean_x = float(np.nonzero(green_near)[1].mean())
            comp_mean_x = float(centroids[label][0])
            if green_mean_x < comp_mean_x + self.green_right_margin_px:
                continue

            if self.last_line_x is not None:
                score = -abs(comp_mean_x - self.last_line_x)  # 가까울수록 높은 점수
            else:
                score = float(area)
            if best is None or score > best[0]:
                best = (score, label)

        if best is None:
            reason = (
                'no green-backed line' if shape_pass else 'no line-shaped component'
            )
            return None, None, 'none', reason

        line_mask = (labels == best[1]).astype(np.uint8) * 255
        measured_x, mode = self.measure_near_x(line_mask)
        if measured_x is None:
            return line_mask, None, mode, 'near band empty'
        return line_mask, measured_x, mode, ''

    @staticmethod
    def component_elongation(component_mask):
        """Return rotated long-side/short-side ratio for one component."""
        ys, xs = np.nonzero(component_mask)
        if len(xs) < 3:
            return 1.0
        points = np.column_stack((xs, ys)).astype(np.float32)
        side_a, side_b = cv2.minAreaRect(points)[1]
        long_side = max(float(side_a), float(side_b))
        short_side = min(float(side_a), float(side_b))
        if short_side < 1.0:
            return long_side
        return long_side / short_side

    def measure_near_x(self, line_mask):
        """실선 중 차량과 y축으로 가장 가까운 구간의 x를 median으로 잰다.

        커브에서 실선 전체를 평균 내면 먼 쪽 곡률에 끌려 기준 x가 왜곡되므로,
        ROI 바닥에서 near_rows 이내의 픽셀만 사용한다. 그 구간이 비어 있으면
        (실선이 화면 위쪽에서만 보이는 경우) 실선 자체의 아래쪽 near_rows 행으로
        폴백한다. 어느 쪽을 썼는지 모드로 돌려 디버그에 표시한다.
        """
        height = line_mask.shape[0]
        ys, xs = np.nonzero(line_mask)
        if ys.size == 0:
            return None, 'none'

        near = ys >= (height - self.near_rows)
        if int(near.sum()) >= self.near_min_pixels:
            return float(np.median(xs[near])), 'near band'

        if self.allow_line_bottom_fallback:
            bottom_cut = ys.max() - self.near_rows
            bottom = ys >= bottom_cut
            if int(bottom.sum()) >= self.near_min_pixels:
                return float(np.median(xs[bottom])), 'line bottom'
        return None, 'none'

    def find_center_line(self, white_mask):
        """중앙 점선 조각들을 하나의 트랙으로 묶어 하단 x를 추정한다.

        점선은 위/아래 조각이 연결요소로 분리된다. 한 조각만 고르면 그 조각이
        사라지는 프레임에 fallback이 끊기므로, 기준 x 근처의 세로로 분리된 조각을
        모두 모은 뒤 슬라이딩 윈도우로 빈 구간을 건너며 직선 피팅한다.
        """
        reference_x = (
            self.last_center_line_x
            if self.last_center_line_x is not None
            else self.target_center_x
        )
        center_mask, piece_count, reason = self.collect_center_dashed_pieces(
            white_mask, reference_x
        )
        if center_mask is None:
            return None, None, 'none', reason

        measured_x, observation_count = self.track_center_dashed_line(
            center_mask, reference_x
        )
        if measured_x is None:
            return center_mask, None, 'none', 'center sliding-window track failed'
        return (
            center_mask,
            measured_x,
            f'center track ({piece_count} pieces/{observation_count} points)',
            '',
        )

    def collect_center_dashed_pieces(self, white_mask, reference_x):
        """중앙선 후보 중 위아래로 분리된 점선 조각을 함께 고른다."""
        reconnect_kernel = np.ones(
            (self.center_reconnect_height_px, 1), dtype=np.uint8
        )
        candidates_mask = cv2.morphologyEx(
            white_mask, cv2.MORPH_CLOSE, reconnect_kernel
        )
        height, _width = white_mask.shape
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            candidates_mask, connectivity=8
        )
        candidates = []
        for label in range(1, num_labels):
            x, y, w, h, area = stats[label]
            if area < self.min_component_area or h < self.min_line_height_px:
                continue
            if w > 0 and (h / float(w)) < self.min_line_aspect_ratio:
                continue

            local_y, local_x = np.where(labels == label)
            if len(np.unique(local_y)) >= 3:
                slope, intercept = np.polyfit(local_y, local_x, 1)
                bottom_x = float(slope * (height - 1) + intercept)
            else:
                bottom_x = float(centroids[label][0])
            bbox_distance = max(
                float(x) - float(reference_x),
                float(reference_x) - float(x + w - 1),
                0.0,
            )
            distance = abs(bottom_x - float(reference_x))
            if min(distance, bbox_distance) <= self.center_search_half_width_px:
                candidates.append((distance, label, x, y, w, h))

        if not candidates:
            return None, 0, 'no center dash component in search window'

        # 기준선에 가장 가까운 조각을 시작점으로 정하고, y 구간이 겹치지 않는
        # 위/아래 조각을 같은 점선 트랙으로 추가한다.
        selected = [min(candidates, key=lambda candidate: candidate[0])]
        for candidate in sorted(candidates, key=lambda item: item[0]):
            if candidate[1] == selected[0][1]:
                continue
            _distance, _label, _x, y, _w, h = candidate
            separate = True
            for chosen in selected:
                chosen_y, chosen_h = chosen[3], chosen[5]
                overlap = max(
                    0, min(y + h, chosen_y + chosen_h) - max(y, chosen_y)
                )
                if overlap / float(min(h, chosen_h)) > self.center_max_vertical_overlap_ratio:
                    separate = False
                    break
            if separate:
                selected.append(candidate)
            if len(selected) >= self.center_max_tracked_pieces:
                break

        filtered = np.zeros_like(white_mask)
        for _distance, label, _x, _y, _w, _h in selected:
            # 닫힘(morphological close)으로 후보를 연결했더라도, offset에는 실제
            # 흰색 점선 픽셀만 사용한다. 점선 사이의 가상 연결선에 끌리지 않는다.
            filtered[(labels == label) & (white_mask > 0)] = 255
        return filtered, len(selected), ''

    def track_center_dashed_line(self, center_mask, base_x):
        """점선 공백을 유지하며 모든 관측 조각으로 하단 x를 피팅한다."""
        height, width = center_mask.shape
        window_height = max(1, height // self.center_num_windows)
        x_current = float(base_x)
        collected_x, collected_y = [], []

        for index in range(self.center_num_windows):
            y_high = height - index * window_height
            y_low = max(0, height - (index + 1) * window_height)
            x_low = max(0, int(round(x_current - self.center_window_margin_px)))
            x_high = min(width, int(round(x_current + self.center_window_margin_px + 1)))
            if x_high <= x_low or y_high <= y_low:
                continue

            sub_mask = center_mask[y_low:y_high, x_low:x_high]
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
                sub_mask, connectivity=8
            )
            choices = [
                label for label in range(1, num_labels)
                if stats[label, cv2.CC_STAT_AREA]
                >= self.center_window_min_component_pixels
            ]
            if not choices:
                # 점선 사이 공백에서는 현재 위치를 유지해 다음 조각을 계속 찾는다.
                continue

            label = min(
                choices,
                key=lambda item: abs((centroids[item][0] + x_low) - x_current),
            )
            local_y, local_x = np.where(labels == label)
            xs, ys = local_x + x_low, local_y + y_low
            x_current = float(xs.mean())
            collected_x.extend(xs.tolist())
            collected_y.extend(ys.tolist())

        if len(collected_x) < self.center_window_min_component_pixels:
            return None, len(collected_x)
        if len(set(collected_y)) >= 3 and len(collected_x) >= 12:
            slope, intercept = np.polyfit(collected_y, collected_x, 1)
            line_x = float(slope * (height - 1) + intercept)
        else:
            line_x = float(np.mean(collected_x))
        return line_x, len(collected_x)

    def map_line_x_to_offset(self, measured_x, target_x):
        """검출한 기준선 x를 해당 목표 x로 되돌리는 offset(+/-45)을 만든다.

        measured_x < target(차가 오른쪽으로 치우침) -> error<0 -> offset<0 -> 좌조향.
        오른쪽 실선과 중앙 점선은 x만 다를 뿐 같은 화면 좌표/부호 규약을 쓴다.
        """
        error_px = float(measured_x) - float(target_x)
        normalized = np.clip(error_px / self.offset_error_limit_px, -1.0, 1.0)
        scaled = normalized * self.lane_offset_limit
        return int(round(np.clip(
            scaled, -self.lane_offset_limit, self.lane_offset_limit
        )))

    def publish_offset(self, value):
        self.output = int(np.clip(value, -self.lane_offset_limit, self.lane_offset_limit))

    def publish_debug(
        self, frame, status, line_mask, measured_x, measure_mode,
        raw_offset=None, target_x=None, line_kind='RIGHT', held_x=None,
    ):
        if not (self.debug_view or self.publish_debug_image):
            return

        debug = frame.copy()
        height, width = debug.shape[:2]
        if target_x is None:
            target_x = self.target_right_x
        # BEV 전체가 검출 ROI다.
        roi_top_y = 0
        roi_bottom_y = height - 1

        # ROI 경계(노랑)
        cv2.rectangle(debug, (0, roi_top_y), (width - 1, roi_bottom_y), (0, 255, 255), 1)

        # 조향 x를 재는 근접 밴드(초록 반투명) — "어느 정도 가까운 지점만 보는지"
        near_top_y = int(np.clip(height - self.near_rows, 0, height - 1))
        overlay = debug.copy()
        cv2.rectangle(
            overlay, (0, near_top_y), (width - 1, roi_bottom_y), (0, 220, 0), -1
        )
        cv2.addWeighted(overlay, 0.28, debug, 0.72, 0, debug)
        cv2.line(debug, (0, near_top_y), (width - 1, near_top_y), (0, 220, 0), 1)
        cv2.putText(
            debug, f'near_rows={self.near_rows}', (8, near_top_y - 6),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1, cv2.LINE_AA,
        )

        # 인식된 오른쪽 실선 "전체"를 빨강으로 칠한다(마스킹 정확도 확인용)
        if line_mask is not None and line_mask.any():
            ys, xs = np.nonzero(line_mask)
            debug[ys, xs] = (0, 0, 255)

        # 기준 x 세로선(주황) — "여기 오면 offset=0"
        self.draw_dashed_vline(
            debug, target_x, roi_top_y, roi_bottom_y, (0, 165, 255)
        )
        cv2.putText(
            debug,
            f'target_{line_kind.lower()}_x={target_x}',
            (min(target_x + 6, width - 190), roi_top_y + 18),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2, cv2.LINE_AA,
        )

        # 측정된 x(노란 원 + 세로선)와 기준선까지의 오차
        if measured_x is not None:
            mx = int(round(measured_x))
            cv2.circle(debug, (mx, roi_bottom_y - 4), 7, (0, 255, 255), -1)
            self.draw_dashed_vline(
                debug, mx, near_top_y, roi_bottom_y, (0, 255, 255), dash=5
            )
            cv2.line(
                debug, (mx, roi_bottom_y - 4),
                (int(target_x), roi_bottom_y - 4), (255, 255, 255), 1,
            )

        # 이번 프레임의 중앙선은 신뢰하지 않아 offset을 유지하는 중이다. 직전
        # 정상 중앙선 위치는 보라색으로 남겨, 제어가 어느 위치를 유지하는지
        # 디버그 화면에서도 바로 확인할 수 있게 한다.
        if held_x is not None:
            hx = int(round(held_x))
            cv2.circle(debug, (hx, roi_bottom_y - 4), 8, (255, 0, 255), 2)
            self.draw_dashed_vline(
                debug, hx, near_top_y, roi_bottom_y, (255, 0, 255), dash=4
            )
            cv2.putText(
                debug, f'HELD center_x={held_x:.0f}',
                (min(hx + 6, width - 180), near_top_y + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2, cv2.LINE_AA,
            )

        color = (0, 255, 0) if status in (
            'OK', 'CENTER FALLBACK', 'CENTER STEP LIMITED'
        ) else (0, 0, 255)
        err = '--' if measured_x is None else f'{measured_x - target_x:+.0f}'
        lines = [
            f'status: {status}   mode: {measure_mode}',
            f'{line_kind.lower()}_x: {"--" if measured_x is None else f"{measured_x:.0f}"} '
            f'-> target {target_x} (err {err})',
            (
                f'held_center_x: {held_x:.0f} (previous accepted)'
                if held_x is not None else ''
            ),
            f'offset: {raw_offset if raw_offset is not None else "--"} '
            f'(smoothed {self.last_offset:.1f})',
            f'green: min_px={self.green_min_pixels} near={self.green_near_distance_px}',
        ]
        for i, text in enumerate(lines):
            cv2.putText(
                debug, text, (10, 20 + i * 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA,
            )

        self.result = {
            'overlay': debug, 'status': status, 'line_kind': line_kind,
            'measured_x': measured_x, 'target_x': target_x, 'held_x': held_x,
            'target': self.output, 'right_stable_count': self.right_green_stable_count,
            'error_px': None if measured_x is None else measured_x - target_x,
            'white_mask': self.white_mask, 'green_mask': self.green_mask,
        }

    def draw_dashed_vline(self, img, x, y_top, y_bottom, color, dash=8):
        x = int(x)
        if x < 0 or x >= img.shape[1]:
            return
        for y in range(y_top, y_bottom, dash * 2):
            cv2.line(img, (x, y), (x, min(y + dash, y_bottom)), color, 2)
