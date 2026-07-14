namespace Hmi.Models
{
	/// <summary>
	/// On-disk root object for <c>*.teach.json</c> files written by
	/// <see cref="WaypointStore.SaveTo(string)"/> and read by
	/// <see cref="WaypointStore.LoadFrom(string)"/>.
	///
	/// Kept as a plain DTO with public settable properties so
	/// <see cref="System.Web.Script.Serialization.JavaScriptSerializer"/> can
	/// round-trip via reflection. Do NOT rename or drop fields — bump
	/// <see cref="WaypointStore.CurrentSchemaVersion"/> and add a migration
	/// step if the schema needs to change.
	/// </summary>
	public sealed class WaypointFile
	{
		public int SchemaVersion { get; set; }
		public WaypointDto[] Waypoints { get; set; }
	}

	/// <summary>
	/// Wire-format for a single taught pose. Mirrors the mutable subset of
	/// <see cref="Waypoint"/>: <c>Motion</c> is serialized as the string "L" /
	/// "J" (rather than the enum's numeric value) so files stay human-readable
	/// even under a future enum reorder.
	/// </summary>
	public sealed class WaypointDto
	{
		public string   Name        { get; set; }
		public double[] PoseXyzAbc  { get; set; }
		public string   Motion      { get; set; }
		public double   VRatio      { get; set; }
		public double   ARatio      { get; set; }
		public double   BlendRadius { get; set; }
	}
}
