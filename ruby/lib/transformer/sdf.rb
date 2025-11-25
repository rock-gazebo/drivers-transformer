require 'sdf'

module Transformer
    module SDF
        # Load a SDF model or file and convert it into a transformer configuration
        def load_sdf(model, exclude_models: [], &producer_resolver)
            model = ::SDF::Root.load(model, flatten: false)
            parse_sdf_root(model, exclude_models: exclude_models, &producer_resolver)
        end

        def parse_sdf_root(sdf, exclude_models: [], &producer_resolver)
            sdf.each_world do |w|
                parse_sdf_world(w, exclude_models: exclude_models, &producer_resolver)
            end

            sdf.each_model do |m|
                next if exclude_models.include?(m.name) || exclude_models.include?(m)

                parse_sdf_model(m, "", "", &producer_resolver)
            end
        end

        def parse_sdf_world(sdf, exclude_models: [], &producer_resolver)
            world_full_name = sdf.full_name
            frames world_full_name
            sdf.each_model do |m|
                next if exclude_models.include?(m.name) || exclude_models.include?(m)

                parse_sdf_model(m, "", world_full_name, world_name: world_full_name, &producer_resolver)
            end
        end

        def sdf_append_name(prefix, name)
            if prefix.empty? then name
            else "#{prefix}::#{name}"
            end
        end

        # Define transformations according to the given SDF model
        #
        # @param [SDF::Model] sdf
        # @param [#[]] filter an object that will be called `filter[link, link_full_name]`
        #   and should return true if the link should be included in the transformer
        #   definitions or not. If nil, all links are included.
        def parse_sdf_model(
            sdf, prefix = "", parent_name = "", filter: nil, world_name: nil,
            &producer_resolver
        )
            full_name = sdf_append_name(prefix, sdf.name)
            frames full_name

            parse_sdf_links_and_joints(
                sdf, full_name, full_name,
                filter: filter, world_name: world_name,
                &producer_resolver
            )

            if world_name
                begin
                    TransformationManager.new(self).resolve_static_chain(full_name, world_name)
                rescue TransformationNotFound
                    if canonical = sdf.canonical_link
                        frame_name = sdf_append_name(full_name, canonical.name)
                        pose = sdf_link_pose_in_world(canonical)
                    else
                        frame_name = full_name
                        pose = sdf.pose
                    end

                    if sdf.static?
                        static_transform(*pose, frame_name => parent_name)
                    else
                        example_transform(*pose, frame_name => parent_name)
                    end
                end
            end
        end

        def sdf_link_pose_in_world(model)
            pose = Eigen::Isometry3.Identity
            until model.kind_of?(::SDF::World)
                pose = model.pose * pose
                model = model.parent
            end
            pose
        end

        def sdf_pose_in_model(object, sdf)
            pose = Eigen::Isometry3.Identity
            while object != sdf
                pose = object.pose * pose
                object = object.parent
            end
            pose
        end

        # Define frames for a model and its canonical link
        def sdf_add_root_model(sdf, prefix = "")
            return unless (canonical_link = sdf.canonical_link)

            static_transform(
                Eigen::Vector3.Zero,
                sdf_append_name(prefix, "#{sdf.name}::#{canonical_link.name}") =>
                    sdf_append_name(prefix, sdf.name)
            )
        end

        # @api private
        #
        # Define frames for links, and either static, dynamic or example transformations
        # for joints
        #
        # @param [SDF::Model] sdf
        # @param [#[]] filter an object that will be called `filter[link, link_full_name]`
        #   and should return true if the link should be included in the transformer
        #   definitions or not. If nil, all links are included.
        # @param [#call] producer_resolver if given, it will be called with a joint
        #   (a SDF::Joint) and should return a producer for the joint's transformation
        #   In that case, the joint will be represented as a dynamic transform
        def parse_sdf_links_and_joints(
            sdf, prefix = "", parent_name = "",
            filter: nil, world_name: nil, &producer_resolver
        )
            submodel2model = {}
            submodel2model[sdf] = Eigen::Isometry3.Identity
            sdf.each_model_with_name do |submodel, _|
                submodel2model[submodel] = sdf_pose_in_model(submodel, sdf)
            end

            relative_link_names = {}
            excluded_links = Set.new
            sdf.each_link_with_name do |l, l_name|
                full_name = sdf_append_name(prefix, l_name)

                if filter && !filter[l, full_name]
                    excluded_links << l
                    next
                end

                frames << full_name
                relative_link_names[l] = l_name
            end

            world_link = ::SDF::Link::World

            if (canonical_link = sdf.canonical_link)
                static_transform(
                    Eigen::Vector3.Zero,
                    sdf_append_name(prefix, canonical_link.name) => parent_name
                )
            end

            sdf.each_joint_with_name do |j, j_name|
                parent = j.parent_link
                child  = j.child_link
                next if excluded_links.include?(parent) || excluded_links.include?(child)

                if parent == ::SDF::Link::World
                    child2model  = submodel2model[child.parent] * child.pose
                    child2parent = sdf_link_pose_in_world(child)
                elsif child == ::SDF::Link::World
                    child2model  = Eigen::Isometry3.Identity
                    child2parent = sdf_link_pose_in_world(parent).inverse
                else
                    parent2model = submodel2model[parent.parent] * parent.pose
                    child2model  = submodel2model[child.parent] * child.pose
                    child2parent = parent2model.inverse * child2model
                end
                joint2child  = j.pose
                joint2parent = child2parent * joint2child

                if j.type != 'fixed'
                    axis_limit = j.axis.limit
                    upper = axis_limit.upper || 0
                    lower = axis_limit.lower || 0
                    axis = j.axis.xyz
                    if j.axis.use_parent_model_frame?
                        # The axis is expressed in the parent model frame ...
                        # Convert to joint frame
                        joint2model = child2model * joint2child
                        axis = joint2model.rotation.inverse * axis
                    end
                    post2pre = j.transform_for((upper + lower) / 2, nil, axis)
                end

                # Handle the special 'world' link
                parent_frame_name =
                    if world_name && (parent == world_link)
                        world_name
                    else
                        sdf_append_name(parent_name, relative_link_names[parent])
                    end

                child_frame_name =
                    if world_name && (child == world_link)
                        world_name
                    else
                        sdf_append_name(parent_name, relative_link_names[child])
                    end

                if upper == lower
                    static_transform child2parent, child_frame_name => parent_frame_name
                else
                    joint_pre  = sdf_append_name(parent_name, "#{j_name}_pre")
                    joint_post = sdf_append_name(parent_name, "#{j_name}_post")
                    register_joint(joint_post, joint_pre, j)
                    static_transform(joint2child, joint_post => child_frame_name)
                    static_transform(joint2parent, joint_pre => parent_frame_name)
                    if producer_resolver && (p = producer_resolver.call(j))
                        dynamic_transform p, joint_post => joint_pre
                    end
                    example_transform post2pre, joint_post => joint_pre
                end
            end
        end
    end
    Transformer::Configuration.include SDF
end
