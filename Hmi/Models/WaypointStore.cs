using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Globalization;
using System.IO;
using System.Text;
using System.Web.Script.Serialization;

namespace Hmi.Models
{
	/// <summary>
	/// Owns the observable collection of taught waypoints plus the mutation ops
	/// the Teach tab UI needs (Add / RemoveAt / MoveUp / MoveDown / Clear / Rename
	/// / CaptureFromPose / NextAutoName) and the JSON persistence entry points
	/// (SaveTo / LoadFrom) that back the File ▸ Teach Pendant menu.
	///
	/// No UI / no OCCT here — the store stays a pure data owner. Serialization
	/// lives on the store (rather than a separate class) because the roadmap's
	/// public API contract names it here; the schema DTO itself is factored out
	/// into <see cref="WaypointFile"/> / <see cref="WaypointDto"/> so future
	/// format migrations have somewhere to grow.
	/// </summary>
	public sealed class WaypointStore
	{
		/// <summary>Version stamp written into every saved file. Bump on breaking schema change.</summary>
		public const int CurrentSchemaVersion = 1;

		/// <summary>Canonical file extension for waypoint files (with leading dot).</summary>
		public const string FileExtension = ".teach.json";
		/// <summary>
		/// Live collection bound by the DataGrid in 3.7.2 and iterated by
		/// <c>WaypointSequencePlayer</c> in 3.10. Mutate through the store's
		/// methods; direct <see cref="ObservableCollection{T}"/> mutation is also
		/// legal (the store keeps no derived state).
		/// </summary>
		public ObservableCollection<Waypoint> Waypoints { get; } = new ObservableCollection<Waypoint>();

		public int Count => Waypoints.Count;

		/// <summary>Append a fully-constructed waypoint. No-op on <c>null</c>.</summary>
		public void Add( Waypoint wp )
		{
			if( wp == null ) return;
			Waypoints.Add( wp );
		}

		/// <summary>
		/// Snapshot the given TCP pose into a new waypoint (defaults: MotionType.L,
		/// V/A ratio = 1.0, blend = 0) and append. <paramref name="name"/> null → auto.
		/// Returns the new waypoint, or <c>null</c> if <paramref name="pose"/> is
		/// invalid (null or shorter than 6).
		/// </summary>
		public Waypoint CaptureFromPose( double[] pose, MotionType motion = MotionType.L, string name = null )
		{
			if( pose == null || pose.Length < Waypoint.PoseElementCount ) return null;
			var wp = new Waypoint {
				Name = string.IsNullOrEmpty( name ) ? NextAutoName() : name,
				Motion = motion
			};
			wp.SetPose( pose );
			Waypoints.Add( wp );
			return wp;
		}

		public bool RemoveAt( int index )
		{
			if( index < 0 || index >= Waypoints.Count ) return false;
			Waypoints.RemoveAt( index );
			return true;
		}

		/// <summary>Swap with the previous row. No-op at index 0 / out-of-range.</summary>
		public bool MoveUp( int index )
		{
			if( index <= 0 || index >= Waypoints.Count ) return false;
			Waypoints.Move( index, index - 1 );
			return true;
		}

		/// <summary>Swap with the next row. No-op at last row / out-of-range.</summary>
		public bool MoveDown( int index )
		{
			if( index < 0 || index >= Waypoints.Count - 1 ) return false;
			Waypoints.Move( index, index + 1 );
			return true;
		}

		public void Clear() => Waypoints.Clear();

		public bool Rename( int index, string newName )
		{
			if( index < 0 || index >= Waypoints.Count ) return false;
			Waypoints[ index ].Name = newName ?? string.Empty;
			return true;
		}

		/// <summary>
		/// Returns the next <c>P{N}</c> name where <c>N</c> is one greater than the
		/// max integer suffix currently in the list. Empty list → "P1". Non
		/// P-prefixed names are ignored, so operators can rename freely without
		/// tripping the auto-numberer.
		/// </summary>
		public string NextAutoName()
		{
			int maxN = 0;
			for( int i = 0; i < Waypoints.Count; i++ ) {
				var n = Waypoints[ i ].Name;
				if( string.IsNullOrEmpty( n ) || n[ 0 ] != 'P' ) continue;
				if( int.TryParse( n.Substring( 1 ), NumberStyles.Integer, CultureInfo.InvariantCulture, out int v ) && v > maxN ) {
					maxN = v;
				}
			}
			return "P" + ( maxN + 1 ).ToString( CultureInfo.InvariantCulture );
		}

		// -------------------------------------------------------------------
		// JSON save / load (3.9)
		// -------------------------------------------------------------------
		// Throws on any failure (I/O error, malformed JSON, schema mismatch).
		// LoadFrom validates the full file into a staged list before touching
		// the observable collection, so a bad file leaves the store's current
		// contents untouched — the DataGrid never sees a half-loaded state.
		// MainWindow's handler translates exceptions into the status bar +
		// MessageBox.Warning UX called out in the roadmap.

		/// <summary>
		/// Serialize the current queue to <paramref name="path"/> as UTF-8 JSON.
		/// Overwrites any existing file. Throws on I/O failure.
		/// </summary>
		public void SaveTo( string path )
		{
			if( string.IsNullOrEmpty( path ) ) throw new ArgumentException( "Path cannot be empty.", nameof( path ) );

			var file = new WaypointFile {
				SchemaVersion = CurrentSchemaVersion,
				Waypoints     = new WaypointDto[ Waypoints.Count ]
			};
			for( int i = 0; i < Waypoints.Count; i++ ) {
				var wp = Waypoints[ i ];
				// Deep-copy the pose array so post-save edits to the live Waypoint
				// can't mutate what we're about to serialize on a background write.
				var pose = new double[ Waypoint.PoseElementCount ];
				Array.Copy( wp.PoseXyzAbc, pose, Waypoint.PoseElementCount );
				file.Waypoints[ i ] = new WaypointDto {
					Name        = wp.Name,
					PoseXyzAbc  = pose,
					Motion      = wp.Motion.ToString(),
					VRatio      = wp.VRatio,
					ARatio      = wp.ARatio,
					BlendRadius = wp.BlendRadius
				};
			}

			var json = new JavaScriptSerializer().Serialize( file );
			File.WriteAllText( path, json, new UTF8Encoding( encoderShouldEmitUTF8Identifier: false ) );
		}

		/// <summary>
		/// Load waypoints from <paramref name="path"/>, replacing the current
		/// queue. Two-phase: parse + validate into a staged list first; only
		/// commits to <see cref="Waypoints"/> after every entry passed
		/// validation, so a partially-bad file is a full no-op. Throws
		/// <see cref="InvalidDataException"/> on schema errors, other exceptions
		/// on I/O / JSON errors.
		/// </summary>
		public void LoadFrom( string path )
		{
			if( string.IsNullOrEmpty( path ) ) throw new ArgumentException( "Path cannot be empty.", nameof( path ) );

			var json = File.ReadAllText( path, Encoding.UTF8 );
			WaypointFile file;
			try {
				file = new JavaScriptSerializer().Deserialize<WaypointFile>( json );
			} catch( Exception ex ) {
				// JavaScriptSerializer throws ArgumentException / InvalidOperationException
				// on malformed JSON. Wrap so the handler can present a consistent message.
				throw new InvalidDataException( "File is not valid JSON: " + ex.Message, ex );
			}
			if( file == null ) throw new InvalidDataException( "File is empty or not a JSON object." );
			if( file.Waypoints == null ) throw new InvalidDataException( "Missing 'Waypoints' array in file." );

			var staged = new List<Waypoint>( file.Waypoints.Length );
			for( int i = 0; i < file.Waypoints.Length; i++ ) {
				var dto = file.Waypoints[ i ];
				if( dto == null ) {
					throw new InvalidDataException( $"Waypoint {i} entry is null." );
				}
				if( dto.PoseXyzAbc == null || dto.PoseXyzAbc.Length < Waypoint.PoseElementCount ) {
					throw new InvalidDataException(
						$"Waypoint {i} ('{dto.Name}') has invalid PoseXyzAbc (expected {Waypoint.PoseElementCount} doubles)." );
				}
				if( !TryParseMotion( dto.Motion, out MotionType motion ) ) {
					throw new InvalidDataException(
						$"Waypoint {i} ('{dto.Name}') has invalid Motion '{dto.Motion}' (expected 'L' or 'J')." );
				}

				var wp = new Waypoint {
					Name        = dto.Name ?? string.Empty,
					Motion      = motion,
					// Setters clamp; explicit assignment preserves the values from disk
					// where legal, or snaps to the legal range otherwise (no data loss
					// vs a hard reject, since the ratios only affect speed).
					VRatio      = dto.VRatio,
					ARatio      = dto.ARatio,
					BlendRadius = dto.BlendRadius
				};
				wp.SetPose( dto.PoseXyzAbc );
				staged.Add( wp );
			}

			// Commit phase: replace store atomically now that we know every entry
			// deserialized cleanly. Callers observing Waypoints via INPC on the
			// collection see one Clear + N Adds rather than a mid-parse mess.
			Waypoints.Clear();
			for( int i = 0; i < staged.Count; i++ ) Waypoints.Add( staged[ i ] );
		}

		private static bool TryParseMotion( string s, out MotionType motion )
		{
			if( string.Equals( s, "L", StringComparison.OrdinalIgnoreCase ) ) { motion = MotionType.L; return true; }
			if( string.Equals( s, "J", StringComparison.OrdinalIgnoreCase ) ) { motion = MotionType.J; return true; }
			motion = MotionType.L;
			return false;
		}
	}
}
