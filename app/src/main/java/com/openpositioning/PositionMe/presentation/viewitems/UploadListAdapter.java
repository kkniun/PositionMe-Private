package com.openpositioning.PositionMe.presentation.viewitems;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.ViewGroup;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;

import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.Traj;
import com.openpositioning.PositionMe.presentation.fragment.UploadFragment;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.HashMap;
import java.io.File;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Adapter used for displaying local Trajectory file data
 *
 * @see UploadViewHolder corresponding View Holder class
 * @see com.openpositioning.PositionMe.R.layout#item_upload_card_view xml layout file
 *
 * @author Mate Stodulka
 */
public class UploadListAdapter extends RecyclerView.Adapter<UploadViewHolder> {

    private final Context context;
    private final List<File> uploadItems;
    private final DownloadClickListener listener;
    private final Map<String, String> trajectoryNameCache = new HashMap<>();

    /**
     * Default public constructor with context for inflating views and list to be displayed.
     *
     * @param context       application context to enable inflating views used in the list.
     * @param uploadItems   List of trajectory Files found locally on the device.
     * @param listener      clickListener to download trajectories when clicked.
     *
     * @see com.openpositioning.PositionMe.Traj protobuf objects exchanged with the server.
     */
    public UploadListAdapter(Context context, List<File> uploadItems, DownloadClickListener listener) {
        this.context = context;
        this.uploadItems = uploadItems;
        this.listener = listener;
    }

    /**
     * {@inheritDoc}
     *
     * @see com.openpositioning.PositionMe.R.layout#item_upload_card_view xml layout file
     */
    @NonNull
    @Override
    public UploadViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
        return new UploadViewHolder(LayoutInflater.from(context).inflate(R.layout.item_upload_card_view, parent, false), listener);
    }

    /**
     * {@inheritDoc}
     * Formats and assigns the data fields from the local Trajectory Files object to the TextView fields.
     *
     * @see UploadFragment finding the data from on local storage.
     * @see com.openpositioning.PositionMe.R.layout#item_upload_card_view xml layout file.
     */
    @Override
    public void onBindViewHolder(@NonNull UploadViewHolder holder, int position) {
        holder.trajId.setText(String.valueOf(position + 1));
        Pattern datePattern = Pattern.compile("_(.*?)\\.txt");
        Matcher dateMatcher = datePattern.matcher(uploadItems.get(position).getName());
        String dateString = dateMatcher.find() ? dateMatcher.group(1) : "N/A";
        String trajectoryName = getTrajectoryName(uploadItems.get(position), dateString);
        holder.trajDate.setText(trajectoryName);

        // Set click listener for the delete button
        holder.deletebutton.setOnClickListener(v -> deleteFileAtPosition(position));

    }

    /**
     * {@inheritDoc}
     * Number of local files.
     */
    @Override
    public int getItemCount() {
        return uploadItems.size();
    }

    private void deleteFileAtPosition(int position)
    {
        if (position >= 0 && position < uploadItems.size())
        {
            File fileToDelete = uploadItems.get(position);
            trajectoryNameCache.remove(fileToDelete.getAbsolutePath());

            if (fileToDelete.exists() && fileToDelete.delete())
            {
                uploadItems.remove(position);
                notifyItemRemoved(position);
                notifyItemRangeChanged(position, uploadItems.size()); // Update subsequent items
                Toast.makeText(context, "File deleted successfully", Toast.LENGTH_SHORT).show();
            }
            else
            {
                Toast.makeText(context, "Failed to delete file", Toast.LENGTH_SHORT).show();
            }
        }
    }

    private String getTrajectoryName(File file, String fallback) {
        String cacheKey = file.getAbsolutePath();
        String cached = trajectoryNameCache.get(cacheKey);
        if (cached != null) {
            return cached;
        }

        String name = fallback;
        try (FileInputStream inputStream = new FileInputStream(file)) {
            Traj.Trajectory trajectory = Traj.Trajectory.parseFrom(inputStream);
            if (!trajectory.getTrajectoryName().trim().isEmpty()) {
                name = trajectory.getTrajectoryName().trim();
            } else if (!trajectory.getTrajectoryId().trim().isEmpty()) {
                name = trajectory.getTrajectoryId().trim();
            }
        } catch (IOException ignored) {
            // Fallback to the filename timestamp when the local file cannot be parsed.
        }

        trajectoryNameCache.put(cacheKey, name);
        return name;
    }
}
