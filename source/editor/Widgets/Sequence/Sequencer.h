/*
Copyright(c) 2015-2026 Panos Karabelas

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

//= INCLUDES ====================
#include "../Widget.h"
#include "Sequence.h"
#include <vector>
#include <unordered_set>
//===============================

class Sequencer : public Widget
{
public:
    Sequencer(Editor* editor);

    void OnTickVisible() override;

    void Save();
    void Load();

private:
    void HandleInput();
    void DrawEmptyState();
    void DrawToolbar();
    void DrawTimeline();
    void DrawTrackList();
    void DrawTimelineContent();
    void DrawPlayhead(float timeline_x, float content_width, float timeline_y, float total_height);
    void DrawMinimap(float timeline_x, float content_width, float timeline_y);
    void DrawAddTrackPopup();
    void DrawSelectionProperties();

    float ComputeTotalHeight() const;
    float SnapTime(float time) const;
    std::string GetFilePath() const;
    void ClearSelection();
    void DeleteSelection();
    void DuplicateSelection();

    // the sequencer owns all sequences
    std::vector<spartan::Sequence> m_sequences;
    std::unordered_set<uint64_t> m_collapsed_sequences;

    // timeline view state
    float m_scroll_x        = 0.0f;
    float m_scroll_y        = 0.0f;
    float m_pixels_per_sec  = 100.0f;
    float m_track_height    = 44.0f;
    float m_group_height    = 36.0f;
    float m_global_duration = 10.0f;

    // resizable splitter
    float m_track_list_width = 220.0f;
    bool  m_resizing_splitter = false;

    // snap
    bool  m_snap_enabled  = false;
    float m_snap_interval = 0.25f;

    // selection state
    int32_t m_selected_sequence = -1;
    int32_t m_selected_track    = -1;
    int32_t m_selected_keyframe = -1;
    int32_t m_selected_clip     = -1;

    // multi-select for keyframes
    std::unordered_set<uint32_t> m_selected_keyframes;

    // dragging state
    int32_t m_dragging_keyframe = -1;
    int32_t m_dragging_track    = -1;
    int32_t m_dragging_sequence = -1;
    int32_t m_dragging_event    = -1;
    bool    m_scrubbing         = false;

    // clip edge resize
    int32_t m_resizing_clip     = -1;
    int32_t m_resizing_clip_seq = -1;
    int32_t m_resizing_clip_trk = -1;
    bool    m_resizing_left     = false;

    // box select
    bool  m_box_selecting    = false;
    float m_box_start_x      = 0.0f;
    float m_box_start_y      = 0.0f;
    int32_t m_box_select_seq = -1;
    int32_t m_box_select_trk = -1;

    // track reorder drag
    int32_t m_reorder_seq    = -1;
    int32_t m_reorder_track  = -1;
    float   m_reorder_start_y = 0.0f;
    bool    m_reorder_active = false;

    // inline rename
    int32_t m_renaming_sequence = -1;
    int32_t m_renaming_track    = -1;
    char    m_rename_buf[128]   = {};

    // right-click position
    float m_popup_mouse_x = 0.0f;

    // context menu target
    int32_t m_popup_sequence = -1;
    int32_t m_popup_track    = -1;

    // cached timeline origin for zoom-toward-mouse
    float m_timeline_origin_x = 0.0f;

    // properties panel
    bool m_properties_open = true;
};
