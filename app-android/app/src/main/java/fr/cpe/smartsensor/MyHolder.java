package fr.cpe.smartsensor;

import android.view.View;
import android.widget.ImageView;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;

/**
 * Specific ViewHolder for the RecyclerView
 */
class MyHolder extends RecyclerView.ViewHolder {

    ImageView myImageView;
    TextView mTitle, mDesc;

    MyHolder(@NonNull View itemView) {
        super(itemView);

        this.myImageView = itemView.findViewById(R.id.imageIv);
        this.mTitle = itemView.findViewById(R.id.titleTv);
        this.mDesc = itemView.findViewById(R.id.descriptionTv);
    }
}
